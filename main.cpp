#include <gpio/gpio.h>
#include <usart/usart.h>
#include <interrupt/interrupt.h>
#include <rcc/rcc.h>
#include <usb/usb.h>
#include <usb/descriptor.h>
#include <timer/timer.h>


int main() {
	rcc_init();
	return 0;
}
