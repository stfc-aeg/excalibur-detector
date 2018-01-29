from iocbuilder import Device
from iocbuilder.arginfo import makeArgInfo, Simple

__all__ = ['ExcaliburProcessPlugin']


class ExcaliburProcessPlugin(Device):

    """Store configuration for ExcaliburProcessPlugin."""

    NAME = "excalibur"

    # Device attributes
    AutoInstantiate = True

    def __init__(self, MACRO):
        self.__super.__init__()
        # Update attributes with parameters
        self.__dict__.update(locals())

    ArgInfo = makeArgInfo(__init__,
                          MACRO=Simple('Dependency MACRO as in configure/RELEASE', str))
