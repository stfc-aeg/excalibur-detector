
class ExcaliburDefinitions(object):
    ERROR_OK = 0
    ERROR_FEM = 1
    ERROR_REQUEST = 2

    ALL_FEMS = 0
    ALL_CHIPS = 0

    X_PIXELS_PER_CHIP = 256
    Y_PIXELS_PER_CHIP = 256
    X_CHIPS_PER_FEM = 8
    Y_CHIPS_PER_FEM = 1

    FEM_PIXELS_PER_CHIP = X_PIXELS_PER_CHIP * Y_PIXELS_PER_CHIP

    FEM_TRIGMODE_INTERNAL = 0
    FEM_TRIGMODE_EXTERNAL = 1
    FEM_TRIGMODE_SYNC = 2
    FEM_TRIGMODE_NAMES = ('internal', 'external', 'extsync')

    FEM_READOUT_MODE_SEQUENTIAL = 0
    FEM_READOUT_MODE_CONTINUOUS = 1
    FEM_READOUT_MODE_NAMES = ('sequential', 'continuous')

    FEM_COLOUR_MODE_FINEPITCH = 0
    FEM_COLOUR_MODE_SPECTROSCOPIC = 1
    FEM_COLOUR_MODE_NAMES = ('fine pitch', 'spectroscopic')

    FEM_CSMSPM_MODE_SINGLE = 0
    FEM_CSMSPM_MODE_SUMMING = 1
    FEM_CSMSPM_MODE_NAMES = ('single pixel', 'charge summing')

    FEM_DISCCSMSPM_DISCL = 0
    FEM_DISCCSMSPM_DISCH = 1
    FEM_DISCCSMSPM_NAMES = ('DiscL', 'DiscH')

    FEM_EQUALIZATION_MODE_OFF = 0
    FEM_EQUALIZATION_MODE_ON = 1
    FEM_EQUALIZATION_MODE_NAMES = ('off', 'on')

    FEM_GAIN_MODE_SHGM = 0
    FEM_GAIN_MODE_HGM = 1
    FEM_GAIN_MODE_LGM = 2
    FEM_GAIN_MODE_SLGM = 3
    FEM_GAIN_MODE_NAMES = ('SHGM', 'HGM', 'LGM', 'SLGM')

    FEM_TEST_PULSE_DISABLE = 0
    FEM_TEST_PULSE_ENABLE = 1
    FEM_TEST_PULSE_NAMES = ('disable', 'enable')

    FEM_OPERATION_MODE_NORMAL = 0
    FEM_OPERATION_MODE_BURST = 1
    FEM_OPERATION_MODE_HISTOGRAM = 2
    FEM_OPERATION_MODE_DACSCAN = 3
    FEM_OPERATION_MODE_MAXTRIXREAD = 4
    FEM_OPERATION_MODE_NAMES = ('normal', 'burst', 'histogram', 'dac scan', 'matrix read')

    FEM_LFSR_BYPASS_MODE_DISABLED = 0
    FEM_LFSR_BYPASS_MODE_ENABLED = 1
    FEM_LFSR_BYPASS_MODE_NAMES = ('disabled', 'enabled')

    FEM_COUNTER_DEPTH_MAP = {'1': 0, '6': 1, '12': 2, '24': 3}

    @classmethod
    def _resolve_mode_name(cls, mode, names):
        try:
            name = names[mode]
        except:
            name = 'unknown'
        return name

    @classmethod
    def trigmode_name(cls, mode):
        return cls._resolve_mode_name(mode, cls.FEM_TRIGMODE_NAMES)

    @classmethod
    def readout_mode_name(cls, mode):
        return cls._resolve_mode_name(mode, cls.FEM_READOUT_MODE_NAMES)

    @classmethod
    def colour_mode_name(cls, mode):
        return cls._resolve_mode_name(mode, cls.FEM_COLOUR_MODE_NAMES)

    @classmethod
    def csmspm_mode_name(cls, mode):
        return cls._resolve_mode_name(mode, cls.FEM_CSMSPM_MODE_NAMES)

    @classmethod
    def disccsmspm_name(cls, mode):
        return cls._resolve_mode_name(mode, cls.FEM_DISCCSMSPM_NAMES)

    @classmethod
    def equalisation_mode_name(cls, mode):
        return cls._resolve_mode_name(mode, cls.FEM_EQUALIZATION_MODE_NAMES)

    @classmethod
    def gain_mode_name(cls, mode):
        return cls._resolve_mode_name(mode, cls.FEM_GAIN_MODE_NAMES)

    @classmethod
    def operation_mode_name(cls, mode):
        return cls._resolve_mode_name(mode, cls.FEM_OPERATION_MODE_NAMES)

    @classmethod
    def lfsr_bypass_mode_name(cls, mode):
        return cls._resolve_mode_name(mode, cls.FEM_LFSR_BYPASS_MODE_NAMES)

    @classmethod
    def counter_depth(cls, depth):
        return cls.FEM_COUNTER_DEPTH_MAP[depth] if depth in cls.FEM_COUNTER_DEPTH_MAP else -1

