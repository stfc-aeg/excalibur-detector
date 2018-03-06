from __future__ import unicode_literals, absolute_import
from future.utils import raise_with_traceback

import logging

from io import StringIO
from collections import OrderedDict
from configparser import SafeConfigParser
from excalibur.definitions import ExcaliburDefinitions
from excalibur.config_parser import ExcaliburDacConfigParser, ExcaliburPixelConfigParser, ExcaliburThresholdConfigParser


class CalibrationError(Exception):
    pass


class DetectorCalibration(object):
    FILE_PIXEL_MASK = 'pixelmask.chip'
    FILE_DISCL_BITS = 'discLbits.chip'
    FILE_DISCH_BITS = 'discHbits.chip'
    CHIPS = [1, 2, 3, 4, 5, 6, 7, 8]

    def __init__(self):
        self._file_root = None
        self._csm_spm_mode = None
        self._gain_mode = None
        self._energy_threshold = None
        self._dacs = {}
        self._masks = {}
        self._discL = {}
        self._discH = {}
        self._thresh0 = {}
        self._thresh1 = {}

    def set_file_root(self, file_root):
        self._file_root = file_root

    def set_csm_spm_mode(self, mode):
        self._csm_spm_mode = mode

    def set_gain_mode(self, mode):
        self._gain_mode = mode

    def set_energy_threshold(self, threshold):
        self._energy_threshold = threshold

    def get_dac(self, fem):
        # Check if we need to update the dac from the energy threshold
        dac = self._dacs[fem]
        if fem in self._thresh0:
            thresh = self._thresh0[fem]
            if thresh.gains is not None and thresh.offsets is not None:
                for chip in self.CHIPS:
                    #logging.debug("Updating DAC value [fem %d chip %d]", fem, chip)
                    #logging.debug("Gain [%s]", thresh.gains[chip - 1])
                    #logging.debug("Offset [%s]", thresh.offsets[chip - 1])
                    val = int(round(thresh.gains[chip - 1] * self._energy_threshold + thresh.offsets[chip - 1]))
                    #logging.debug("Updating DAC value [fem %d chip %d] to %s", fem, chip, val)
                    dac.update_dac_value(fem, chip, 'Threshold0', val)
        if fem in self._thresh1:
            thresh = self._thresh1[fem]
            if thresh.gains is not None and thresh.offsets is not None:
                for chip in self.CHIPS:
                    #logging.debug("Updating DAC value [fem %d chip %d]", fem, chip)
                    #logging.debug("Gain [%s]", thresh.gains[chip - 1])
                    #logging.debug("Offset [%s]", thresh.offsets[chip - 1])
                    val = int(round(thresh.gains[chip - 1] * self._energy_threshold + thresh.offsets[chip - 1]))
                    #logging.debug("Updating DAC value [fem %d chip %d] to %s", fem, chip, val)
                    dac.update_dac_value(fem, chip, 'Threshold1', val)
        return dac

    def get_mask(self, fem):
        return self._masks[fem]

    def get_discH(self, fem):
        return self._discH[fem]

    def get_discL(self, fem):
        return self._discL[fem]

    def cal_file_root(self, fem):
        # Check for the file root and construct the ini path from this
        if self._file_root is None:
            raise CalibrationError("No calibration file root has been set")

        filename = self._file_root + '/fem' + str(fem) + '/'

        # Check for csmspm mode and append the relevant value to the path
        if self._csm_spm_mode is None:
            raise CalibrationError("There is no CSM PSM mode selected")

        if self._csm_spm_mode == ExcaliburDefinitions.FEM_CSMSPM_MODE_SINGLE:
            filename += 'spm/'
        elif self._csm_spm_mode == ExcaliburDefinitions.FEM_CSMSPM_MODE_SUMMING:
            filename += 'csm/'

        # Check for gain mode and append the relevant value to the path
        if self._gain_mode is None:
            raise CalibrationError("There is no Gain mode selected")

        if self._gain_mode == ExcaliburDefinitions.FEM_GAIN_MODE_SHGM:
            filename += 'shgm/'
        elif self._gain_mode == ExcaliburDefinitions.FEM_GAIN_MODE_HGM:
            filename += 'hgm/'
        elif self._gain_mode == ExcaliburDefinitions.FEM_GAIN_MODE_LGM:
            filename += 'lgm/'
        elif self._gain_mode == ExcaliburDefinitions.FEM_GAIN_MODE_SLGM:
            filename += 'slgm/'

        return filename

    def load_calibration_files(self, fems):
        if isinstance(fems, list):
            for fem in fems:
                self._dacs[fem] = self.load_dac_calibration_file(fem)
                self._masks[fem] = self.load_pixel_calibration_file(fem, self.FILE_PIXEL_MASK, self.CHIPS)
                self._discH[fem] = self.load_pixel_calibration_file(fem, self.FILE_DISCH_BITS, self.CHIPS)
                self._discL[fem] = self.load_pixel_calibration_file(fem, self.FILE_DISCL_BITS, self.CHIPS)
                self._thresh0[fem] = self.load_threshold_calibration_file(fem, 0)
                self._thresh1[fem] = self.load_threshold_calibration_file(fem, 1)
        else:
            self._dacs[fems] = self.load_dac_calibration_file(fems)
            self._masks[fems] = self.load_pixel_calibration_file(fems, self.FILE_PIXEL_MASK, self.CHIPS)
            self._discH[fems] = self.load_pixel_calibration_file(fems, self.FILE_DISCH_BITS, self.CHIPS)
            self._discL[fems] = self.load_pixel_calibration_file(fems, self.FILE_DISCL_BITS, self.CHIPS)
            self._thresh0[fems] = self.load_threshold_calibration_file(fems, 0)
            self._thresh1[fems] = self.load_threshold_calibration_file(fems, 1)

    def manual_dac_calibration(self, fems, filename):
        if isinstance(fems, list):
            for fem in fems:
                self._dacs[fem] = self.load_dac_calibration_file(fem, filename)
        else:
            self._dacs[fems] = self.load_dac_calibration_file(fems, filename)

    def manual_mask_calibration(self, fems, filename):
        if isinstance(fems, list):
            for fem in fems:
                self._masks[fem] = self.load_pixel_calibration_file(fem, self.FILE_PIXEL_MASK, self.CHIPS, filename)
        else:
            self._masks[fems] = self.load_pixel_calibration_file(fems, self.FILE_PIXEL_MASK, self.CHIPS, filename)

    def load_dac_calibration_file(self, fem, filename=None):
        if filename is None:
            filename = self.cal_file_root(fem) + 'dacs'
        return ExcaliburDacConfigParser([filename], [fem], [1, 2, 3, 4, 5, 6, 7, 8])

    def load_pixel_calibration_file(self, fem, type, chips, filename=None):
        config = []
        if filename is None:
            filename = self.cal_file_root(fem) + type
            for chip in chips:
                config.append(ExcaliburPixelConfigParser(filename + str(chip-1)))
                logging.debug("Pixel file loaded for chip %d => %s...", chip, config[chip-1].pixels[0:20])
        else:
            for chip in chips:
                config.append(ExcaliburPixelConfigParser(filename))
                logging.debug("Pixel file loaded for chip %d => %s...", chip, config[chip - 1].pixels[0:20])
        return config

    def load_threshold_calibration_file(self, fem, threshold):
        filename = self.cal_file_root(fem) + 'threshold' + str(threshold)
        config = ExcaliburThresholdConfigParser(filename)
        logging.debug("Threshold file loaded gains   => %s", config.gains)
        logging.debug("                      offsets => %s", config.offsets)
        return config

    # def update_dacs_from_thresholds(self, fems):
    #     if self._energy_threshold is None:
    #         raise CalibrationError("No energy threshold value has been set")
    #
    #     for fem in fems:
    #         # If threshold0 exists then replace the corresponding dac values
    #         if fem in self._thresh0:
    #             thresh = self._thresh0[fem]
    #             if thresh.gains is not None and thresh.offsets is not None:
    #                 for chip in self.CHIPS:
    #                     val = int(round(thresh.gains[chip-1] * self._energy_threshold + thresh.offsets[chip-1]))
    #                     self.get_dac(fem).
    #                 Threshold0

# class DacCalibrationFile(object):
#     """
#     Loads a DAC calibration INI file.
#     """
#     def __init__(self):
#         self._ini_filename = None
#         self._conf = None
#
#     def load_ini(self, ini_filename):
#         """
#         Loads and parses the data from INI file. The data is stored internally in the object and can be retrieved
#         through the property methods
#         """
#         self._ini_filename = ini_filename
#         self._conf = SafeConfigParser(dict_type=OrderedDict)
#         self._conf.optionxform = str
#         if self._ini_filename:
#             self._conf.read(self._ini_filename)
#             logging.info("Read DAC calibration INI file: %s", self._ini_filename)
#
#     @property
#     def sections(self):
#         return self._conf.sections()
#
#     def get(self, chip, item):
#         section = "CHIP" + str(chip)
#         for data in self._conf.items(section):
#             if item == data[0]:
#                 return int(data[1])
#         return None
#
#     def get_description(self, section):
#         desc = ""
#         for item in self._conf.items(section):
#             if "Setpoint_description" in item[0]:
#                 desc = item[1].replace('"', '')
#                 break
#         return desc
#
#     def get_setpoints(self, section):
#         sps = {}
#         for item in self._conf.items(section):
#             if "Setpoint_description" not in item and "Setpoint_name" not in item:
#                 sps[item[0]] = item[1]
#         return sps


def main():
    logging.getLogger().setLevel(logging.DEBUG)
#    cb = DacCalibrationFile()
#    cb.load_ini('/dls_sw/i13-1/epics/excalibur/V2.7/fem1/spm/hgm/dacs')

    cb = DetectorCalibration()
    cb.set_file_root('/dls_sw/i13-1/epics/excalibur/V2.7')
    cb.set_csm_spm_mode(0)
    cb.set_gain_mode(0)
    cb.load_calibration_files(1)

#    logging.debug("Sections: %s", cb.sections)
#    logging.debug("Threshold0 => %d", cb.get(1, 'Threshold0'))
#    logging.debug("DACDiscH => %d", cb.get(7, 'DACDiscH'))


if __name__ == '__main__':
    main()
