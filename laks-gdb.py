import gdb

def get_mmio_ptr_type(pt):
    if pt.name.startswith('mmio_ptr<'):
        return pt.template_argument(0)

    for field in pt.fields():
        if not field.is_base_class:
            continue

        t = get_mmio_ptr_type(field.type)

        if t is not None:
            return t

    return None

class MMIOPtr(gdb.Function):
  def __init__(self):
    gdb.Function.__init__(self, 'mmio_ptr')

  def invoke(self, obj):
    t = get_mmio_ptr_type(obj.type)
    return obj['p'].cast(t.pointer())

MMIOPtr()
