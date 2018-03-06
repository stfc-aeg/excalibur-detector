'''
Created on Apr 3 2017

@author= tcn45
'''

from collections import OrderedDict
import ConfigParser
import logging
import os
import re

from excalibur.definitions import ExcaliburDefinitions
    
class ExcaliburDacConfiguration(OrderedDict):
    
    def __init__(self, fem=0, chip=0):
        
        super(ExcaliburDacConfiguration, self).__init__()
    
        self.fem = 0
        self.chip = 0
        self['Threshold0'] = 128 
        self['Threshold1'] = 0
        self['Threshold2'] = 0
        self['Threshold3'] = 0
        self['Threshold4'] = 0
        self['Threshold5'] = 0
        self['Threshold6'] = 0
        self['Threshold7'] = 0
        self['Preamp'] = 100
        self['Ikrum'] = 10
        self['Shaper'] = 125
        self['Disc'] = 125
        self['DiscLS'] = 100
        self['ShaperTest'] = 100
        self['DiscL'] = 68
        self['Test'] = 100
        self['DiscH'] = 90
        self['Delay'] = 50
        self['TPBufferIn'] = 128
        self['TPBufferOut'] = 128
        self['RPZ'] = 255
        self['GND'] = 124
        self['TPREF'] = 128
        self['FBK'] = 178
        self['Cas'] = 170
        self['TPREFA'] = 417
        self['TPREFB'] = 417
        
        
class ExcaliburDacConfigIniParser(object):
    
    def __init__(self, config, fem, chips):
        
        self.dac_param_transforms = {
            'DiscL' : 'DACDiscL',
            'DiscH' : 'DACDiscH',
            'Test' : 'DacTest',
            'TPBufferIn' : 'TPBuffIn',
            'TPBufferOut': 'TPBuffOut',
        }
        
        parser = ConfigParser.SafeConfigParser()
        
        try:
            with open(config) as fp:
                parser.readfp(fp)
        except IOError as e:
            logging.error('Failed to parse DAC configuration file: {}'.format(e))
            raise IOError('Failed to parse DAC configuration file: {}'.format(e))

        if chips == 0 or chips == [0]:
            self._chips = range(1,8+1)
        else:
            self._chips = chips
            
        self._dacs = [] 

        for (idx, chip) in enumerate(self._chips):

            self._dacs.append(ExcaliburDacConfiguration(fem, chip))
            
            chip_section = 'CHIP{}'.format(chip)
            for dac in self._dacs[idx]:
                if dac in self.dac_param_transforms:
                    dac_name = self.dac_param_transforms[dac]
                else:
                    dac_name = dac
                if parser.has_option(chip_section, dac_name):
                    self._dacs[idx][dac] = parser.getint(chip_section, dac_name)

    @property
    def dacs(self):
        return self._dacs
    
    @property
    def chips(self):
        return self._chips
                           
class ExcaliburDacConfigParser(object):
    
    #dac_format_ext = ['.dacs']
    #hdf_format_ext = ['.h5', '.hdf', '.hdf5']
    
    def __init__(self, configs=None, fems=[0], chips=[0]):
        
        self._dacs =  [[ExcaliburDacConfiguration()]]
        self.fems = fems
        self.chips = chips
        
        if configs is None or configs == []:
            
            logging.debug('No/empty list of configuration files specified, using defaults')

        else:
            
            #file_exts = [os.path.splitext(config)[1] for config in configs]
            #num_dac_format_configs = sum([ext in self.dac_format_ext for ext in file_exts])
            #num_hdf_format_configs = sum([ext in self.hdf_format_ext for ext in file_exts])

            #if num_dac_format_configs == 0 and num_hdf_format_configs == 0:
            #    logging.error('No recognised configuration file types, using defaults')
            #if num_dac_format_configs > 0 and num_hdf_format_configs > 0:
            #    logging.error('Cannot mix HDF and DAC format configuration files, using defaults')
            #elif num_dac_format_configs > 0:
            logging.debug('Loading DAC config files')

            if len(configs) == 1:
                logging.debug('Using {} configuration for all FEMs'.format(configs[0]))
                configs = [configs[0]]*len(fems)

            if len(configs) != len(fems):
                logging.error('Mismatch between number of config files and number of FEMs specified, using defaults')
            else:

                self._dacs = [ExcaliburDacConfigIniParser(config, fem, chips) for (config, fem) in zip(configs, fems)]
            #else:
            #    logging.debug('Parsing of HDF format DAC configuration files not yet supported, using defaults')
                
    def dacs(self, fem, chip):
        
        if self.fems == [0]:
            fem_idx = 0
        else:
            fem_idx = self.fems.index(fem)
            
        if self.chips == [0]:
            chip_idx = self._dacs[fem_idx].chips.index(chip)
        else:
            chip_idx = self.chips.index(chip)

        return self._dacs[fem_idx].dacs[chip_idx]

    def update_dac_value(self, fem, chip, param, value):
        if self.fems == [0]:
            fem_idx = 0
        else:
            fem_idx = self.fems.index(fem)

        if self.chips == [0]:
            chip_idx = self._dacs[fem_idx].chips.index(chip)
        else:
            chip_idx = self.chips.index(chip)

        self._dacs[fem_idx].dacs[chip_idx][param] = value

    def dac_api_params(self):
        
        for dac_name in ExcaliburDacConfiguration():
            dac_param = 'mpx3_{}dac'.format(dac_name.lower())
            yield dac_name, dac_param
            
            
class ExcaliburPixelConfigParser(object):
    
    def __init__(self, config_file, expected_size=ExcaliburDefinitions.FEM_PIXELS_PER_CHIP):
        
        self._pixels = None
        self.expected_size = expected_size    

        self._pixels = []
        try:
            with open(config_file) as config_fp:
                for line in config_fp:
                    self._pixels.extend([int(val) for val in line.strip().split(' ')])
        except Exception as e:
            logging.error('Failed to parse pixel configuration mask file: {}'.format(e))
            raise IOError('Failed to parse pixel configuration mask file: {}'.format(e))

    @property
    def pixels(self):
        if self._pixels is None or len(self._pixels) != self.expected_size:
            return [0] * self.expected_size
        else:
            return self._pixels


class ExcaliburThresholdConfigParser(object):
    def __init__(self, config_file, expected_size=8):
        self._expected_size = expected_size
        self._gains = []
        self._offsets = []

        try:
            with open(config_file) as config_fp:
                self._gains.extend([float(val) for val in config_fp.readline().strip().split(' ')])
                self._offsets.extend([float(val) for val in config_fp.readline().strip().split(' ')])
        except Exception as e:
            self._gains = None
            self._offsets = None
            logging.debug('Unable to parse threshold file: {}'.format(e))

    @property
    def gains(self):
        return self._gains

    @property
    def offsets(self):
        return self._offsets
