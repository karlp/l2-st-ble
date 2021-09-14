#include <gpio/gpio.h>
#include <uart/uart.h>
#include <os/time.h>
#include <interrupt/interrupt.h>
#include <rcc/rcc.h>
#include <usb/usb.h>
#include <usb/descriptor.h>
#include <timer/timer.h>

#include <cstdio>
#include <coroutine>
#include <optional>
#include <chrono>
#include <array>
#include <span>
#include <string_view>
#include <tuple>
#include <format>

#include "gen.h"
#include "stubs.h"

using namespace std::chrono_literals;

struct critical_section {
    uint32_t primask;

    critical_section() {
        asm volatile("mrs %0, primask" : "=r" (primask));

        asm volatile("cpsid i");

        asm volatile("dmb");
    }

    ~critical_section() {
        asm volatile("dmb");

        asm volatile("msr primask, %0" :: "r" (primask));
    }
};

struct raii_test {
    raii_test() {
        printf("raii_test()\n");
    }

    ~raii_test() {
        printf("~raii_test()\n");
    }
};

#include <async/async.h>
#include <async/scheduler.h>

struct async_flag : public schedulable {
    bool ready;

    async_flag() : ready(false) {}

    bool await_ready() {
        return ready;
    }

    bool await_suspend(std::coroutine_handle<> h) {
        critical_section lock;

        if(ready) {
            return false;
        } else {
            awaiter = h;
            return true;
        }
    }

    void await_resume() {
        critical_section lock;

        awaiter = nullptr;
        ready = false;
    }

    void set() {
        if(ready) {
            return;
        }

        ready = true;

        if(awaiter) {
            scheduler.schedule(*this);
        }
    }
};

template <typename Duration = std::chrono::milliseconds>
struct time_scheduler_t {
    struct wakeup_future : public schedulable {
        time_scheduler_t& tsched;
        Duration time;
        wakeup_future(time_scheduler_t& tsched, Duration time) : tsched(tsched), time(time) {}

        bool await_ready() {
            return tsched.now >= time;
        }

        bool await_suspend(std::coroutine_handle<> h) {
            if(tsched.now >= time) {
                return false;
            } else {
                awaiter = h;
                tsched.push_wakeup(this);
                return true;
            }
        }

        void await_resume() {}

        void resume() {
            scheduler.schedule(*this);
        }
    };

    Duration now;

    wakeup_future* wakeup_queue;

    void push_wakeup(wakeup_future* fut) {
        wakeup_future** p = &wakeup_queue;

        while((*p) && (*p)->time <= fut->time) {
            p = reinterpret_cast<wakeup_future**>(&(*p)->next);
        }

        fut->next = *p;
        *p = fut;
    }

    async_flag tick_flag;

    void tick() {
        tick_flag.set();
    }

    task wakeup_task() {
        while(1) {
            co_await tick_flag;
            now++;

            while(wakeup_queue && wakeup_queue->time <= now) {
                auto fut = wakeup_queue;
                wakeup_queue = static_cast<wakeup_future*>(fut->next);
                fut->resume();
            }
        }
    }

    wakeup_future sleep_until(Duration time) {
        return {*this, time};
    }

    wakeup_future sleep(Duration duration) {
        return sleep_until(now + duration);
    }
};

time_scheduler_t time_scheduler;

template <>
void interrupt::handler<interrupt::exception::SysTick>() {
    time_scheduler.tick();
}

template <typename T, std::size_t N>
struct queue {
    std::array<T, N> data;
    volatile std::size_t put_idx;
    volatile std::size_t get_idx;

    async_flag put_flag;
    async_flag get_flag;

    constexpr std::size_t _inc(std::size_t n) {
        return n + 1 < N ? n + 1 : 0;
    }

    bool empty() {
        return put_idx == get_idx;
    }

    bool full() {
        return _inc(put_idx) == get_idx;
    }

    bool put(T v) {
        if(full()) {
            return false;
        }

        data[put_idx] = v;
        put_idx = _inc(put_idx);
        put_flag.set();
        return true;
    }

    bool get(T& v) {
        if(empty()) {
            return false;
        }

        v = data[get_idx];
        get_idx = _inc(get_idx);
        get_flag.set();
        return true;
    }

    async<> async_put(T v) {
        while(!put(v)) {
            co_await get_flag;
        }
    }

    async<T> async_get() {
        T v;
        while(!get(v)) {
            co_await put_flag;
        }
        co_return v;
    }
};

gen<int> range(int n) {
    for(int i = 0; i < n; i++) {
        co_yield i;
    }
}

async<int> async_add(int a, int b) {
    co_return a + b;
}

async<int> async_sub(int a, int b) {
    return async_add(a, -b);
}

async<int> async_addsub(int a, int b, int c) {
    a = co_await async_add(a, b);
    a = co_await async_sub(a, c);
    co_return a;
}

async<int> async_add3(int a, int b, int c) {
    a = co_await async_add(a, b);
    a = co_await async_add(a, c);
    co_return a;
}

async<> indirect_yield() {
    co_await yield();
}

task math() {
    printf("3 + 5 = %d\n", co_await async_add(3, 5));

    printf("3 - 2 = %d\n", co_await async_sub(3, 2));

    printf("1 + 2 + 3 = %d\n", co_await async_add3(1, 2, 3));
}

task foo(char c) {
    printf("task %c started\n", c);

    co_await indirect_yield();

    printf("task %c resumed\n", c);
}

queue<char, 8> uart1_rx;
queue<char, 8> uart1_tx;

template <>
void interrupt::handler<interrupt::irq::USART1>() {
    //if(USART1->ISR & (1 << 5)) { // RXNE
    if(USART1->SR & (1 << 5)) { // RXNE
        //char c = USART1->RDR;
        char c = USART1->DR;
        uart1_rx.put(c);
    }

    //if(USART1->ISR & (1 << 7)) { // TXE
    if(USART1->SR & (1 << 7)) { // TXE
        char c;
        if(uart1_tx.get(c)) {
            //USART1->TDR = c;
            USART1->DR = c;
        } else {
            USART1->CR1 &= ~(1 << 7); // TXEIE
        }
    }
}

task uart_reader() {
    char c = 0;

    while(c != 'q') {
        c = co_await uart1_rx.async_get();

        printf("Reader task got: %c\n", c);

        if(c == 'p') {
            printf("Sleeping for 5s.\n");
            co_await time_scheduler.sleep(5s); 
        }
    }
}

task await_and_print(async<int> coro) {
    printf("Awaited: %d\n", co_await coro);
}

async<> sleep(std::chrono::milliseconds period) {
    auto start = Time::time();

    while(Time::time() - start < period.count()) {
        co_await yield();
    }
}

task periodic(char c, std::chrono::milliseconds period) {
    while(1) {
        printf("%c: Now: %d ms.\n", c, int(time_scheduler.now.count()));
        printf("%c: Sleeping for %d ms.\n", c, int(period.count()));
        co_await time_scheduler.sleep(period);
        printf("%c: Slept for %d ms.\n", c, int(period.count()));
    }
}

struct shell_io_t {
    async<char> getchar() {
        return uart1_rx.async_get();
    }

    async<> putchar(char c) {
        co_await uart1_tx.async_put(c);
        //USART1.reg.CR1 |= 1 << 7; // TXEIE
        USART1->CR1 |= 1 << 7; // TXEIE
    }

    async<> print(std::string_view s) {
        for(auto c : s) {
            co_await putchar(c);
        }
    }

    template<typename ...T>
    async<> print(T... args) {
        //std::array<char, 128> buf;
        //auto n = snprintf(buf.data(), buf.size(), args...);
        //co_await print(std::string_view(buf.data(), n));

        auto s = std::format(args...);
        co_await print(std::string_view(s));
    }
};

shell_io_t shell_io;

struct shell_cmd {
    std::string_view name;

    async<> (*cmd)(shell_io_t&);
};

std::string_view substr_nothrow(std::string_view v, std::size_t pos) {
    return {v.data() + pos, v.size() - pos};
}

std::string_view substr_nothrow(std::string_view v, std::size_t pos, std::size_t count) {
    return {v.data() + pos, count};
}

std::tuple<std::string_view, std::string_view> split_first(std::string_view v) {
    auto space = std::min(v.find(' '), v.size());
    auto space_end = std::min(v.find_first_not_of(' ', space), v.size());

    //auto first = v.substr(0, space);
    auto first = substr_nothrow(v, 0, space);
    //auto remaining = v.substr(space_end);
    auto remaining = substr_nothrow(v, space_end);

    return {first, remaining};
}

task shell_task(shell_io_t& io, std::span<shell_cmd> cmds) {
    std::array<char, 80> buf;
    std::size_t idx = 0;

    while(1) {
        char c = co_await io.getchar();

        if(c >= 0x20) {
            if(idx >= buf.size()) {
                continue;
            }
            buf[idx++] = c;
            co_await io.putchar(c);
        }

        if(c == '\r') {
            if(!idx) {
                continue;
            }

            co_await io.putchar('\n');

            auto [name, args] = split_first(std::string_view(buf.data(), idx));

            bool found = false;

            for(auto cmd : cmds) {
                if(name != cmd.name) {
                    continue;
                }

                co_await cmd.cmd(io);
                found = true;
                break;
            }

            if(!found) {
                buf[name.size()] = 0;
                co_await io.print("No such command: {}\n", name);
            }

            idx = 0;
        }
    }
}

shell_cmd shell_cmds[] = {
    {
        "foo",
        [](auto io) -> async<> {
            co_await io.print("Hello world!\n");
        },
    },
};

//typedef task (*task_fp)();
using task_fp = task (*)();

struct ITM_reg_t {
    volatile uint32_t STIM[256];
    uint32_t _reserved[640];
    volatile uint32_t TER[8];
    uint32_t _reserved2[8];
    volatile uint32_t TPR;
    uint32_t _reserved3[15];
    volatile uint32_t TCR;
    uint32_t _reserved4[76];
    volatile uint32_t LSR;
};

struct DWT_reg_t {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
    volatile uint32_t CPICNT;
    volatile uint32_t EXCCNT;
    volatile uint32_t SLEEPCNT;
    volatile uint32_t LSUCNT;
    volatile uint32_t FOLDCNT;
    volatile uint32_t PCSR;
};

struct TPIU_reg_t {
    volatile uint32_t SSPSR;
    volatile uint32_t CSPSR;
    uint32_t _reserved[2];
    volatile uint32_t ACPR;
    uint32_t _reserved2[55];
    volatile uint32_t SPPR;
};

constexpr mmio_ptr<ITM_reg_t> ITM {0xe0000000};
constexpr mmio_ptr<DWT_reg_t> DWT {0xe0001000};
constexpr mmio_ptr<TPIU_reg_t> TPIU {0xe0040000};

auto led_red = GPIOD[12];
auto led_yellow = GPIOD[13];
auto led_green = GPIOD[14];

task led_task() {
    led_red.set_mode(Pin::Output);
    led_yellow.set_mode(Pin::Output);
    led_green.set_mode(Pin::Output);

    uint32_t i = 0;

    while(1) {
        ITM->STIM[5] = 'R';
        ITM->STIM[6] = i++;
        led_red.on();
        co_await time_scheduler.sleep(1000ms);
        led_red.off();

        ITM->STIM[5] = 'Y';
        ITM->STIM[6] = i++;
        led_yellow.on();
        co_await time_scheduler.sleep(1000ms);
        led_yellow.off();

        ITM->STIM[5] = 'G';
        ITM->STIM[6] = i++;
        led_green.on();
        co_await time_scheduler.sleep(1000ms);
        led_green.off();
    }
}

uint32_t led_bit_cnt = 0;

template <>
void interrupt::handler<interrupt::irq::TIM2>() {
    TIM2.SR = 0;
    
    switch(++led_bit_cnt) {
        case 12:
        case 28:
        case 68:
        case 76:
        case 84:
            TIM2.CCR1 = 52;
            break;
        case 16:
        case 32:
        case 72:
        case 80:
        case 88:
            TIM2.CCR1 = 26;
            break;
        case 96:
            TIM2.CCR1 = 0;
            break;
        case 1000:
            TIM2.CCR1 = 26;
            led_bit_cnt = 0;
            break;
    }

}

auto dev_desc = device_desc(0x200, 0, 0, 0, 64, 0x1234, 0x5678, 0x110, 1, 2, 3, 1);
auto conf_desc = configuration_desc(1, 1, 0, 0xc0, 0,
	// HID interface.
	interface_desc(0, 0, 1, 0xff, 0x00, 0x00, 0,
		endpoint_desc(0x81, 0x03, 16, 1)
	)
);

desc_t dev_desc_p = {sizeof(dev_desc), (void*)&dev_desc};
desc_t conf_desc_p = {sizeof(conf_desc), (void*)&conf_desc};

USB_otg usb(OTG_HS, dev_desc_p, conf_desc_p);

task usb_task() {
    usb.init();

    OTG_HS->GCCFG |= 1 << 21;

    while(1) {
        usb.process();
        co_await yield();
    }
}

task itm_task() {
    while(1) {
        ITM->STIM[0] = 0xff0055aa;
        ITM->STIM[1] = 0xff0055aa;
        ITM->STIM[2] = 0xff0055aa;
        ITM->STIM[3] = 0xff0055aa;
        co_await yield();
    }
}

void rcc_init(uint32_t osc_mhz, uint32_t sysclk_mhz);

int main() {
    rcc_init(12, 168);

	// Initialize system timer.
	//STK.LOAD = 72000000 / 1000; // 1000 Hz.
	STK.LOAD = 168000000 / 1000 / 8; // 1000 Hz.
	STK.CTRL = 0x03;

    RCC.enable(rcc::GPIOA);
    RCC.enable(rcc::GPIOB);
    RCC.enable(rcc::GPIOD);
    RCC.enable(rcc::GPIOE);
    RCC.enable(rcc::USART1);
    RCC.enable(rcc::OTGHS);

    RCC.enable(rcc::TIM2);

    GPIOA[0].set_af(1);
    GPIOA[0].set_mode(Pin::AF);

    TIM2.ARR = (84000000 / 800000) - 1;
    TIM2.CCMR1 = (6 << 4) | (1 << 3);
    TIM2.CCER = 1 << 0;
    TIM2.DIER = 1 << 0;
    TIM2.CR1 = 1 << 0;

    interrupt_ctl.enable(interrupt::irq::TIM2);

    //GPIOE[2].set_mode(Pin::AF);
    //GPIOE[3].set_mode(Pin::AF);
    //GPIOE[4].set_mode(Pin::AF);
    //GPIOE[5].set_mode(Pin::AF);
    //GPIOE[6].set_mode(Pin::AF);

    //*(volatile uint32_t*)0xe0042004 |= (3 << 6) | (1 << 5); // DBGMCU_CR: TRACE_MODE=4b, TRACE_IOEN

    GPIOB[14].set_af(12);
    GPIOB[15].set_af(12);
    GPIOB[14].set_mode(Pin::AF);
    GPIOB[15].set_mode(Pin::AF);

    //GPIOA[9].set_mode(Pin::AF);
    //GPIOA[10].set_mode(Pin::Input);

    USART1->BRR = 8000000 / 115200;
    USART1->CR1 = (1 << 13) | (1 << 3) | (1 << 2); // UE, RE, TE
    //USART1->CR1 = (1 << 3) | (1 << 2) | (1 << 0); // UE, RE, TE
    USART1->CR1 |= 1 << 5; // RXNEIE

    interrupt_ctl.enable(interrupt::irq::USART1);

    printf("Hello world!\n");

    for(int i : range(5)) {
        printf("%d\n", i);
    }

    printf("gen done\n");

    auto wakeup_task = time_scheduler.wakeup_task();
    auto task_math = math();
    auto task_a = foo('a');
    auto task_b = foo('b');
    auto task_c = foo('c');
    //auto task_uart = uart_reader();
    auto task_add = await_and_print(async_add(8, 1));
    //auto task_periodic_3s = periodic('X', 3s);
    //auto task_periodic_1200ms = periodic('Y', 1200ms);
    auto task_shell = shell_task(shell_io, shell_cmds);
    auto task_led = led_task();
    auto task_usb = usb_task();
    //auto task_itm = itm_task();

    printf("scheduling tasks\n");

    scheduler.schedule(wakeup_task);
    scheduler.schedule(task_math);
    scheduler.schedule(task_a);
    scheduler.schedule(task_b);
    scheduler.schedule(task_c);
    //scheduler.schedule(task_uart);
    scheduler.schedule(task_add);
    //scheduler.schedule(task_periodic_3s);
    //scheduler.schedule(task_periodic_1200ms);
    scheduler.schedule(task_shell);
    scheduler.schedule(task_led);
    //scheduler.schedule(task_usb);
    //scheduler.schedule(task_itm);

    printf("scheduled all tasks\n");

    while(1) {

        //printf("starting scheduler\n");
        scheduler.run();
        //printf("scheduler done\n");

        //asm volatile("ecall");

        {
            critical_section lock;

            if(!scheduler.first) {
                //printf("wfi\n");
                asm volatile("wfi");
            }
        }

        //printf("wakeup\n");
    }
}
