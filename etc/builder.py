from iocbuilder import Device
from iocbuilder.arginfo import makeArgInfo, Simple

__all__ = ['ExcaliburDetector']


class ExcaliburDetector(Device):

    """Store configuration for Excalibur Detector."""

    NAME = "excalibur"

    # Device attributes
    AutoInstantiate = True

    def __init__(self, MACRO):
        self.__super.__init__()
        # Update attributes with parameters
        self.__dict__.update(locals())

    ArgInfo = makeArgInfo(__init__,
                          MACRO=Simple('Dependency MACRO as in configure/RELEASE', str))
