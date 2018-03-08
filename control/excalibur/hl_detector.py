"""
interface_wrapper.py - EXCALIBUR high level API for the ODIN server.

Alan Greer, DLS
"""
import logging
import json
from datetime import datetime
import time
import threading

from excalibur.detector import ExcaliburDetector, ExcaliburDetectorError
from excalibur.calibration_files import DetectorCalibration
from excalibur.definitions import ExcaliburDefinitions
from enum import Enum
from collections import OrderedDict



class ExcaliburParameter(OrderedDict):
    def __init__(self, param, value,
                 fem=ExcaliburDefinitions.ALL_FEMS, chip=ExcaliburDefinitions.ALL_CHIPS):
        super(ExcaliburParameter, self).__init__()
        self['param'] = param
        self['value'] = value
        self['fem'] = fem
        self['chip'] = chip

    def get(self):
        return self.param, self.value, self.fem, self.chip


class ExcaliburReadParameter(OrderedDict):
    def __init__(self, param, fem=ExcaliburDefinitions.ALL_FEMS, chip=ExcaliburDefinitions.ALL_CHIPS):
        super(ExcaliburReadParameter, self).__init__()
        self['param'] = param
        self['fem'] = fem
        self['chip'] = chip

    def get(self):
        return self.param, self.fem, self.chip


class ParameterType(Enum):
    """Enumeration of all available types
    """
    UNKNOWN = 0
    DICT = 1
    LIST = 2
    INT = 3
    DOUBLE = 4
    STRING = 5
    ENUM = 6


class Parameter(object):
    def __init__(self, name, data_type=ParameterType.UNKNOWN, value=None, callback=None):
        self._name = name
        self._datatype = data_type
        self._value = value
        self._callback = callback

    @property
    def value(self):
        return self.get()['value']

    def get(self):
        # Create the dictionary of information
        return_value = {'value': self._value,
                        'type': self._datatype
                        }
        return return_value

    def set_value(self, value):
        if self._value != value:
            self._value = value
            if self._callback is not None:
                self._callback(self._name, self._value)


class EnumParameter(Parameter):
    def __init__(self, name, value=None, allowed_values=None, callback=None):
        super(EnumParameter, self).__init__(name, data_type=ParameterType.ENUM, value=value, callback=callback)
        self._allowed_values = allowed_values

    def get(self):
        # Create the dictionary of information
        return_value = super(EnumParameter, self).get()
        if self._allowed_values is not None:
            return_value['allowed_values'] = self._allowed_values
        return return_value

    @property
    def index(self):
        return self.get()['allowed_values'].index(self.value)


class IntegerParameter(Parameter):
    def __init__(self, name, value=None, limits=None, callback=None):
        super(IntegerParameter, self).__init__(name, data_type=ParameterType.INT, value=value, callback=callback)
        self._limits = limits

    def get(self):
        # Create the dictionary of information
        return_value = super(IntegerParameter, self).get()
        if self._limits is not None:
            return_value['limits'] = self._limits
        return return_value


class DoubleParameter(Parameter):
    def __init__(self, name, value=None, limits=None, callback=None):
        super(DoubleParameter, self).__init__(name, data_type=ParameterType.DOUBLE, value=value, callback=callback)
        self._limits = limits

    def get(self):
        # Create the dictionary of information
        return_value = super(DoubleParameter, self).get()
        if self._limits is not None:
            return_value['limits'] = self._limits
        return return_value


class StringParameter(Parameter):
    def __init__(self, name, value=None, callback=None):
        super(StringParameter, self).__init__(name, data_type=ParameterType.STRING, value=value, callback=callback)


class HLExcaliburDetector(ExcaliburDetector):
    """Wraps the detector class to provide a high level interface.

    """

    def __init__(self, fem_connections):
        super(HLExcaliburDetector, self).__init__(fem_connections)

        # Temporary setup of MAC addresses
#        self.source_data_addr = [u'10.0.2.2']
#        self.source_data_mac = [u'62:00:00:00:00:01']
#        self.source_data_port = [8]
#        self.dest_data_addr = [u'10.0.2.1']
#        self.dest_data_mac = [u'00:07:43:06:31:A7']
#        self.dest_data_mac = [u'A0:36:9F:8C:52:3C']
#        self.dest_data_mac = [u'00:07:43:11:F7:60', u'A0:36:9F:8C:52:3C']
#        self.dest_data_port = [61649]
        self._fems = range(1, len(fem_connections)+1)
        logging.debug("Fem conection IDs: %s", self._fems)

        # Create the calibration object
        self._cb = DetectorCalibration()

        # Create the Excalibur parameters
        self._param = {
            'api': DoubleParameter('api', 0.1),
            'config/num_images': IntegerParameter('num_images', 1),
            'config/exposure_time': DoubleParameter('exposure_time', 1.0),
            'config/num_test_pulses': IntegerParameter('num_test_pulses', 0),
            'config/test_pulse_enable': EnumParameter('test_pulse_enable',
                                                      ExcaliburDefinitions.FEM_TEST_PULSE_NAMES[0],
                                                   ExcaliburDefinitions.FEM_TEST_PULSE_NAMES),
            'config/operation_mode': EnumParameter('operation_mode',
                                                   ExcaliburDefinitions.FEM_OPERATION_MODE_NAMES[0],
                                                   ExcaliburDefinitions.FEM_OPERATION_MODE_NAMES),
            'config/lfsr_bypass': EnumParameter('lfsr_bypass',
                                                ExcaliburDefinitions.FEM_LFSR_BYPASS_MODE_NAMES[0],
                                                ExcaliburDefinitions.FEM_LFSR_BYPASS_MODE_NAMES),
            'config/read_write_mode': EnumParameter('read_write_mode',
                                                    ExcaliburDefinitions.FEM_READOUT_MODE_NAMES[0],
                                                    ExcaliburDefinitions.FEM_READOUT_MODE_NAMES),
            'config/disc_csm_spm': EnumParameter('disc_csm_spm',
                                                 ExcaliburDefinitions.FEM_DISCCSMSPM_NAMES[0],
                                                 ExcaliburDefinitions.FEM_DISCCSMSPM_NAMES),
            'config/equalization_mode': EnumParameter('equalization_mode',
                                                      ExcaliburDefinitions.FEM_EQUALIZATION_MODE_NAMES[0],
                                                      ExcaliburDefinitions.FEM_EQUALIZATION_MODE_NAMES),
            'config/trigger_mode': EnumParameter('trigger_mode',
                                                 ExcaliburDefinitions.FEM_TRIGMODE_NAMES[0],
                                                 ExcaliburDefinitions.FEM_TRIGMODE_NAMES),
            'config/csm_spm_mode': EnumParameter('csm_spm_mode',
                                                 ExcaliburDefinitions.FEM_CSMSPM_MODE_NAMES[0],
                                                 ExcaliburDefinitions.FEM_CSMSPM_MODE_NAMES,
                                                 callback=self.update_calibration),
            'config/colour_mode': EnumParameter('colour_mode',
                                                ExcaliburDefinitions.FEM_COLOUR_MODE_NAMES[0],
                                                ExcaliburDefinitions.FEM_COLOUR_MODE_NAMES),
            'config/gain_mode': EnumParameter('gain_mode',
                                              ExcaliburDefinitions.FEM_GAIN_MODE_NAMES[0],
                                              ExcaliburDefinitions.FEM_GAIN_MODE_NAMES,
                                              callback=self.update_calibration),
            'config/counter_select': IntegerParameter('counter_select', 0),
            'config/counter_depth': EnumParameter('counter_depth',
                                                  '12',
                                                  ['1', '6', '12', '24']),
            'config/cal_file_root': StringParameter('cal_file_root', '', callback=self.update_calibration),
            'config/energy_threshold': DoubleParameter('energy_threshold', 0.0, callback=self.update_calibration),
            'config/udp_file': StringParameter('udp_file', '', callback=self.hl_load_udp_config),
            'config/hv_bias': DoubleParameter('hv_bias', 0.0, callback=self.hl_hv_bias_set),

            #["Normal",
            #                                                                     "Burst",
            #                                                                     "Histogram",
            #                                                                     "DAC Scan",
            #                                                                     "Matrix Read"])
        }
        self._status = {
            'calibration': [0] * len(self._fems),
            'sensor': {
                'width': ExcaliburDefinitions.X_PIXELS_PER_CHIP * ExcaliburDefinitions.X_CHIPS_PER_FEM,
                'height': ExcaliburDefinitions.Y_PIXELS_PER_CHIP *
                          ExcaliburDefinitions.Y_CHIPS_PER_FEM * len(self._fems)
            },
            'manufacturer': 'DLS/STFC',
            'model': 'Odin [Excalibur2]',
            'error': ''
        }
        logging.debug("Status: %s", self._status)
        self._calibration_status = {
            'dac': [0] * len(self._fems),
            'discl': [0] * len(self._fems),
            'disch': [0] * len(self._fems),
            'mask': [0] * len(self._fems),
            'thresh': [0] * len(self._fems)
        }

        self._executing_updates = True
        self._acquiring = False
        self._acq_frame_count = 0
        self._acq_exposure = 0.0
        self._acq_start_time = datetime.now()
        self._acq_timeout = 0.0
        self._comms_lock = threading.RLock()
        self._param_lock = threading.RLock()
        self._fast_update_time = datetime.now()
        self._medium_update_time = datetime.now()
        self._slow_update_time = datetime.now()
        self._frame_start_count = 0
        self._frame_count_time = None
        self._status_thread = threading.Thread(target=self.status_loop)
        self._status_thread.start()

    def hl_load_udp_config(self, name, filename):
        logging.error("Loading UDP configuration [{}] from file {}".format(name, filename))

        try:
            with open(filename) as config_file:
                udp_config = json.load(config_file)
        except IOError as io_error:
            logging.error("Failed to open UDP configuration file: {}".format(io_error))
            return
        except ValueError as value_error:
            logging.error("Failed to parse UDP json config: {}".format(value_error))
            return

        source_data_addr = []
        source_data_mac = []
        source_data_port = []
        dest_data_port_offset = []

        for idx, fem in enumerate(udp_config['fems']):
            source_data_addr.append(fem['ipaddr'])
            source_data_mac.append(fem['mac'])
            source_data_port.append(fem['port'])
            dest_data_port_offset.append(fem['dest_port_offset']
                                         )
            logging.error('    FEM  {:d} : ip {:16s} mac: {:s} port: {:5d} offset: {:d}'.format(
                idx, source_data_addr[-1], source_data_mac[-1],
                source_data_port[-1], dest_data_port_offset[-1]
            ))

        dest_data_addr = []
        dest_data_mac = []
        dest_data_port = []

        for idx, node in enumerate(udp_config['nodes']):
            dest_data_addr.append(node['ipaddr'])
            dest_data_mac.append(node['mac'])
            dest_data_port.append(int(node['port']))

            logging.debug('    Node {:d} : ip {:16s} mac: {:s} port: {:5d}'.format(
                idx, dest_data_addr[-1], dest_data_mac[-1],
                dest_data_port[-1]
            ))

        farm_mode_enable = udp_config['farm_mode']['enable']
        farm_mode_num_dests = udp_config['farm_mode']['num_dests']

        udp_params = []
        num_fems = len(self._fems)
        # Append per-FEM UDP source parameters, truncating to number of FEMs present in system
        udp_params.append(ExcaliburParameter(
            'source_data_addr', [[addr] for addr in source_data_addr[:num_fems]],
        ))
        udp_params.append(ExcaliburParameter(
            'source_data_mac', [[mac] for mac in source_data_mac[:num_fems]],
        ))
        udp_params.append(ExcaliburParameter(
            'source_data_port', [[port] for port in source_data_port[:num_fems]]
        ))
        udp_params.append(ExcaliburParameter(
            'dest_data_port_offset',
            [[offset] for offset in dest_data_port_offset[:num_fems]]
        ))

        # Append the UDP destination parameters, noting [[[ ]]] indexing as they are common for
        # all FEMs and chips - there must be a better way to do this
        udp_params.append(ExcaliburParameter(
            'dest_data_addr', [[[addr for addr in dest_data_addr]]]
        ))
        udp_params.append(ExcaliburParameter(
            'dest_data_mac', [[[mac for mac in dest_data_mac]]]
        ))
        udp_params.append(ExcaliburParameter(
            'dest_data_port', [[[port for port in dest_data_port]]]
        ))

        # Append the farm mode configuration parameters
        udp_params.append(ExcaliburParameter('farm_mode_enable', [[farm_mode_enable]]))
        udp_params.append(ExcaliburParameter('farm_mode_num_dests', [[farm_mode_num_dests]]))

        # Write all the parameters to system
        logging.info('Writing UDP configuration parameters to system')
        self.hl_write_params(udp_params)

    def shutdown(self):
        self._executing_updates = False

    def set_calibration_status(self, fem, status, area=None):
        if area is not None:
            self._calibration_status[area][fem-1] = status
        else:
            for area in ['dac', 'discl', 'disch', 'mask', 'thresh']:
                self._calibration_status[area][fem - 1] = status

        logging.error("Calibration: %s", self._calibration_status)
        bit = 0
        calibration_bitmask = 0
        for area in ['dac', 'discl', 'disch', 'mask', 'thresh']:
            calibration_bitmask += (self._calibration_status[area][fem - 1] << bit)
            bit += 1
        if calibration_bitmask == 0x1F:
            calibration_bitmask += (1 << bit)

        self._status['calibration'][fem-1] = calibration_bitmask

    def hl_manual_dac_calibration(self, filename):
        for fem in self._fems:
            self.set_calibration_status(fem, 0, 'dac')
        self._cb.manual_dac_calibration(self._fems, filename)
        self.download_dac_calibration()
        logging.error("Status: %s", self._status)

    def hl_manual_mask_calibration(self, filename):
        for fem in self._fems:
            self.set_calibration_status(fem, 0, 'mask')
        self._cb.manual_mask_calibration(self._fems, filename)
        self.download_pixel_masks()
        logging.error("Status: %s", self._status)

    def update_calibration(self, name, value):
        logging.debug("Updating calibration due to %s updated to %s", name, value)
        # Reset all calibration status values prior to loading a new calibration
        for fem in self._fems:
            self.set_calibration_status(fem, 0)
        self._cb.set_file_root(self._param['config/cal_file_root'].value)
        self._cb.set_csm_spm_mode(self._param['config/csm_spm_mode'].index)
        self._cb.set_gain_mode(self._param['config/gain_mode'].index)
        self._cb.set_energy_threshold(self._param['config/energy_threshold'].value)
        self._cb.load_calibration_files(self._fems)
        self.download_dac_calibration()
        self.download_pixel_calibration()
        logging.error("Status: %s", self._status)

    def download_dac_calibration(self):
        dac_params = []
        chip_ids = [1, 2, 3, 4, 5, 6, 7, 8]

        for (dac_name, dac_param) in self._cb.get_dac(1).dac_api_params():
            logging.debug("%s  %s", dac_name, dac_param)
            dac_vals = []
            for fem in self._fems:
                fem_vals = [self._cb.get_dac(fem).dacs(fem, chip_id)[dac_name] for chip_id in chip_ids]
                dac_vals.append(fem_vals)

            dac_params.append(ExcaliburParameter(dac_param, dac_vals,
                                                 fem=self._fems, chip=chip_ids))

        # Connect to the hardware
        self.connect({'state': True})

        time.sleep(1.0)

        # Write all the parameters to system
        logging.info('Writing DAC configuration parameters to system {}'.format(str(dac_params)))
        self.write_fe_param(dac_params)

        for fem in self._fems:
            self.set_calibration_status(fem, 1, 'dac')
            self.set_calibration_status(fem, 1, 'thresh')

    def download_pixel_masks(self):
        chip_ids = [1, 2, 3, 4, 5, 6, 7, 8]
        pixel_params = []
        mpx3_pixel_masks = []
        logging.debug("Generating mpx3_pixel_mask...")
        for fem in self._fems:
            fem_vals = [self._cb.get_mask(fem)[chip-1].pixels for chip in chip_ids]
            mpx3_pixel_masks.append(fem_vals)
        pixel_params.append(ExcaliburParameter('mpx3_pixel_mask', mpx3_pixel_masks,
                                               fem=self._fems, chip=chip_ids))

        # Write all the parameters to system
        self.write_fe_param(pixel_params)
        for fem in self._fems:
            self.set_calibration_status(fem, 1, 'mask')

    def download_pixel_calibration(self):
        chip_ids = [1, 2, 3, 4, 5, 6, 7, 8]
        pixel_params = []
        mpx3_pixel_masks = []
        logging.debug("Generating mpx3_pixel_mask...")
        for fem in self._fems:
            fem_vals = [self._cb.get_mask(fem)[chip-1].pixels for chip in chip_ids]
            mpx3_pixel_masks.append(fem_vals)
        pixel_params.append(ExcaliburParameter('mpx3_pixel_mask', mpx3_pixel_masks,
                                               fem=self._fems, chip=chip_ids))

        mpx3_pixel_discl = []
        logging.debug("Generating mpx3_pixel_discl...")
        for fem in self._fems:
            fem_vals = [self._cb.get_discL(fem)[chip-1].pixels for chip in chip_ids]
            mpx3_pixel_discl.append(fem_vals)
        pixel_params.append(ExcaliburParameter('mpx3_pixel_discl', mpx3_pixel_discl,
                                               fem=self._fems, chip=chip_ids))

        mpx3_pixel_disch = []
        logging.debug("Generating mpx3_pixel_disch...")
        for fem in self._fems:
            fem_vals = [self._cb.get_discH(fem)[chip - 1].pixels for chip in chip_ids]
            mpx3_pixel_disch.append(fem_vals)
        pixel_params.append(ExcaliburParameter('mpx3_pixel_disch', mpx3_pixel_disch,
                                               fem=self._fems, chip=chip_ids))

        #pixel_params.append(ExcaliburParameter('mpx3_pixel_test', [[mpx3_pixel_test]],
        #                                       fem=self.args.config_fem,
        #                                       chip=self.args.config_chip))

        # Write all the parameters to system
        self.write_fe_param(pixel_params)
        for fem in self._fems:
            self.set_calibration_status(fem, 1, 'mask')
            self.set_calibration_status(fem, 1, 'discl')
            self.set_calibration_status(fem, 1, 'disch')

    def status_loop(self):
        # Status loop has two polling rates, fast and slow
        # Fast poll is currently set to 0.2 s
        # Slow poll is currently set to 5.0 s
        while self._executing_updates:
            if (datetime.now() - self._slow_update_time).seconds > 5.0:
                self._slow_update_time = datetime.now()
                self.slow_read()
            if (datetime.now() - self._medium_update_time).seconds > 10.0:
                self._medium_update_time = datetime.now()
                self.power_card_read()
            if (datetime.now() - self._fast_update_time).microseconds > 100000:
                self._fast_update_time = datetime.now()
                self.fast_read()
            time.sleep(0.1)

    def get(self, path):
        with self._param_lock:
            if path == 'command/initialise':
                response = {'value': 1}
            elif path == 'command/lv_enable':
                response = {'value': 1}
            elif path == 'command/hv_enable':
                response = {'value': 1}
            elif path == 'command/configure_dac':
                response = {'value': 1}
            elif path == 'command/configure_mask':
                response = {'value': 1}
            elif path in self._param:
                response = self._param[path].get()
            elif self.search_status(path) is not None:
                response = {'value': self.search_status(path)}
                try:
                    response.update(super(HLExcaliburDetector, self).get(path))
                except:
                    # Valid to fail if the get request is for a high level item
                    pass
            else:
                response = super(HLExcaliburDetector, self).get(path)

            return response

    def set(self, path, data):
        try:
            if path in self._param:
                self._param[path].set_value(data)
            elif path == 'command/initialise':
                # Initialise the FEMs
                logging.error('Initialise has been called')
                self.hl_initialise()
            elif path == 'command/lv_enable':
                # Disable/Enable the low voltage
                logging.error('lv_enable has been called')
                self.hl_lv_enable(data)
            elif path == 'command/hv_enable':
                # Disable/Enable the low voltage
                logging.error('hv_enable has been called')
                self.hl_hv_enable(data)
            elif path == 'command/configure_dac':
                # Initialise the FEMs
                logging.error('Manual DAC calibration has been called')
                self.hl_manual_dac_calibration(data)
            elif path == 'command/configure_mask':
                # Initialise the FEMs
                logging.error('Manual mask file download has been called')
                self.hl_manual_mask_calibration(data)
            elif path == 'command/start_acquisition':
                # Starting an acquisition!
                logging.debug('Start acquisition has been called')
                self.do_acquisition()
            elif path == 'command/stop_acquisition':
                # Starting an acquisition!
                logging.debug('Abort acquisition has been called')
                self.do_command('stop_acquisition', None)
            else:
                super(HLExcaliburDetector, self).set(path, data)
        except Exception as ex:
            self.set_error(str(ex))
            raise ExcaliburDetectorError(str(ex))

    def set_error(self, err):
        # Record the error message into the status
        self._status['error'] = err

    def clear_error(self):
        # Record the error message into the status
        self._status['error'] = ""

    def search_status(self, path):
        items = path.split('/')
        item_dict = None
        if items[0] == 'status':
            try:
                item_dict = self._status
                for item in items[1:]:
                    item_dict = item_dict[item]
            except KeyError, ex:
                item_dict = None
        return item_dict

    def fast_read(self):
        status = {}
        frame_rate = 0.0
        with self._comms_lock:
            acq_completion_state_mask = 0x40000000
            # Connect to the hardware
            if not self.connected:
                self.connect({'state': True})

            fem_params = ['frames_acquired', 'control_state']

            read_params = ExcaliburReadParameter(fem_params)
            self.read_fe_param(read_params)

            while True:
                time.sleep(0.01)
                if not self.command_pending():
                    if self._get('command_succeeded'):
                        logging.info("Command has succeeded")
                    else:
                        logging.info("Command has failed")
                    break
            vals = super(HLExcaliburDetector, self).get('command')['command']['fe_param_read']['value']
            logging.info("Raw fast read status: %s", vals)
            # Calculate the minimum number of frames from the fems, as this will be the actual complete frame count
            frames_acquired = min(vals['frames_acquired'])
            #acq_completed = all(
            #    [((state & acq_completion_state_mask) == acq_completion_state_mask) for state in vals['control_state']]
            #)
            if self._acquiring:
                # We are acquiring so check to see if we have the correct number of frames
                if frames_acquired == self._acq_frame_count:
                    self._acquiring = False
                    # Acquisition has finished so we must send the stop command
                    logging.debug("stop_acquisition called at end of a complete acquisition")
                    self.hl_stop_acquisition()
                elif frames_acquired > self._acq_frame_count:
                    # There has been an error in the acquisition, we should never have too many frames
                    self._acquiring = False
                    # Acquisition has finished so we must send the stop command
                    logging.debug("stop_acquisition called at end of a complete acquisition")
                    self.hl_stop_acquisition()
                else:
                    if frames_acquired > 0:
                        if self._frame_count_time is None:
                            self._frame_start_count = frames_acquired
                            self._frame_count_time = datetime.now()
                        # Check to see if we have timed out
                        delta_us = (datetime.now() - self._frame_count_time).microseconds
                        delta_s = (datetime.now() - self._frame_count_time).seconds
                        frame_rate = float(frames_acquired-self._frame_start_count) / (float(delta_s) + (float(delta_us) / 1000000.0))
                    else:
                        self._frame_start_count = 0
                        self._frame_count_time = None
                        frame_rate = 0.0
                    #logging.error("Current frame rate %f", frame_rate)
                    delta_t = (datetime.now() - self._acq_start_time).seconds
                    # Work out the worst case for number of expected frames (assuming 10% plus 1 second startup)
                    delta_t -= 1.0
                    if delta_t > 0.0:
                        expected_frames = int(delta_t / (self._acq_exposure * 1.1))
                        #logging.error("We would have expected %d frames by now", expected_frames)
                        if expected_frames > frames_acquired:
                            self._acquiring = False
                            # Acquisition has finished so we must send the stop command
                            logging.debug("stop_acquisition called due to a timeout")
                            self.hl_stop_acquisition()

            status = {'frames_acquired': frames_acquired,
                      'frame_rate': frame_rate,
                      'acquisition_complete': (not self._acquiring)}
        with self._param_lock:
            self._status.update(status)
        logging.debug("Fast update status: %s", status)

    def power_card_read(self):
        with self._comms_lock:
            # Do not perform a slow read if an acquisition is taking place
            if not self._acquiring:
                # Connect to the hardware
                if not self.connected:
                    self.connect({'state': True})

                powercard_params = ['fe_lv_enable',
                                    'fe_hv_enable',
                                    'pwr_p5va_vmon',
                                    'pwr_p5vb_vmon',
                                    'pwr_p5v_fem00_imon',
                                    'pwr_p5v_fem01_imon',
                                    'pwr_p5v_fem02_imon',
                                    'pwr_p5v_fem03_imon',
                                    'pwr_p5v_fem04_imon',
                                    'pwr_p5v_fem05_imon',
                                    'pwr_p48v_vmon',
                                    'pwr_p48v_imon',
                                    'pwr_p5vsup_vmon',
                                    'pwr_p5vsup_imon',
                                    'pwr_humidity_mon',
                                    'pwr_air_temp_mon',
                                    'pwr_coolant_temp_mon',
                                    'pwr_coolant_flow_mon',
                                    'pwr_p3v3_imon',
                                    'pwr_p1v8_imonA',
                                    'pwr_bias_imon',
                                    'pwr_p3v3_vmon',
                                    'pwr_p1v8_vmon',
                                    'pwr_bias_vmon',
                                    'pwr_p1v8_imonB',
                                    'pwr_p1v8_vmonB',
                                    'pwr_coolant_temp_status',
                                    'pwr_humidity_status',
                                    'pwr_coolant_flow_status',
                                    'pwr_air_temp_status',
                                    'pwr_fan_fault']
                fe_params = powercard_params
                read_params = ExcaliburReadParameter(fe_params, fem=self.powercard_fem_idx+1)
                self.read_fe_param(read_params)

                while True:
                    time.sleep(0.1)
                    if not self.command_pending():
                        if self._get('command_succeeded'):
                            logging.info("Command has succeeded")
                            status = super(HLExcaliburDetector, self).get('command')['command']['fe_param_read'][
                                'value']
                            with self._param_lock:
                                for param in powercard_params:
                                    if param in status:
                                        val = status[param]
                                        if isinstance(val, list):
                                            self._status[param] = val[0]
                                        else:
                                            self._status[param] = val
                        else:
                            logging.error("Command has failed")
                            with self._param_lock:
                                for param in powercard_params:
                                    self._status[param] = None
                        break
        logging.debug("Power card update status: %s", self._status)

    def slow_read(self):
        status = {}
        with self._comms_lock:
            # Do not perform a slow read if an acquisition is taking place
            if not self._acquiring:
                # Connect to the hardware
                if not self.connected:
                    self.connect({'state': True})

                fem_params = ['fem_local_temp', 'fem_remote_temp', 'moly_temp', 'moly_humidity']
                supply_params = ['supply_p1v5_avdd1', 'supply_p1v5_avdd2', 'supply_p1v5_avdd3', 'supply_p1v5_avdd4',
                                'supply_p1v5_vdd1', 'supply_p2v5_dvdd1']

                fe_params = fem_params + supply_params + ['mpx3_dac_out']

                read_params = ExcaliburReadParameter(fe_params)
                self.read_fe_param(read_params)

                while True:
                    time.sleep(0.1)
                    if not self.command_pending():
                        if self._get('command_succeeded'):
                            logging.info("Command has succeeded")
                            status = super(HLExcaliburDetector, self).get('command')['command']['fe_param_read'][
                                'value']
                            with self._param_lock:
                                for param in fe_params:
                                    if param in status:
                                        val = []
                                        if param in supply_params:
                                            for item in status[param]:
                                                if item != 1:
                                                    val.append(0)
                                                else:
                                                    val.append(1)
                                        else:
                                            val = status[param]
                                        self._status[param] = val
                        else:
                            logging.info("Command has failed")
                            with self._param_lock:
                                for param in fe_params:
                                    if param in status:
                                        self._status[param] = status[param]
                        break

        logging.debug("Slow update status: %s", status)

    def do_acquisition(self):
        with self._comms_lock:
            self.clear_error()
            if self._status['frames_acquired'] > 0:
                self.set_error('Detector reports non zero frames, reset detector')
                return
            # Set the acquiring flag
            self._acquiring = True
            self._acq_start_time = datetime.now()
            status = {'acquisition_complete': (not self._acquiring)}
            self._status.update(status)
            # Resolve the acquisition operating mode appropriately, handling burst and matrix read if necessary
            operation_mode = self._param['config/operation_mode']

            # if self.args.burst_mode:
            #     operation_mode = ExcaliburDefinitions.FEM_OPERATION_MODE_BURST
            #
            # if self.args.matrixread:
            #     if self.args.burst_mode:
            #         logging.warning('Cannot select burst mode and matrix read simultaneously, ignoring burst option')
            #     operation_mode = ExcaliburDefinitions.FEM_OPERATION_MODE_MAXTRIXREAD
            #
            # # TODO - handle 24 bit readout here - needs to check frame count etc and execute C0 read

            # Build a list of parameters to be written to the system to set up acquisition
            write_params = []

            tp_count = self._param['config/num_test_pulses'].value
            logging.info('  Setting test pulse count to {}'.format(tp_count))
            write_params.append(ExcaliburParameter('mpx3_numtestpulses', [[tp_count]]))
            tp_enable = self._param['config/test_pulse_enable']
            logging.info('  Setting test pulse enable to {}'.format(tp_enable.value))
            write_params.append(ExcaliburParameter('testpulse_enable', [[tp_enable.index]]))

            num_frames = self._param['config/num_images'].value
            logging.info('  Setting number of frames to {}'.format(num_frames))
            write_params.append(ExcaliburParameter('num_frames_to_acquire', [[num_frames]]))

            # Record the number of frames for this acquisition
            self._acq_frame_count = num_frames

            # Record the acquisition exposure time
            self._acq_exposure = self._param['config/exposure_time'].value

            acquisition_time = int(self._param['config/exposure_time'].value * 1000.0)
            logging.info('  Setting acquisition time to {} ms'.format(acquisition_time))
            write_params.append(ExcaliburParameter('acquisition_time', [[acquisition_time]]))

            trigger_mode = self._param['config/trigger_mode']
            logging.info('  Setting trigger mode to {}'.format(trigger_mode.value))
            write_params.append(ExcaliburParameter('mpx3_externaltrigger', [[trigger_mode.index]]))

            read_write_mode = self._param['config/read_write_mode']
            logging.info('  Setting ASIC readout mode to {}'.format(read_write_mode.value))
            write_params.append(ExcaliburParameter('mpx3_readwritemode', [[read_write_mode.index]]))

            colour_mode = self._param['config/colour_mode']
            logging.info('  Setting ASIC colour mode to {} '.format(colour_mode.value))
            write_params.append(ExcaliburParameter('mpx3_colourmode', [[colour_mode.index]]))

            csmspm_mode = self._param['config/csm_spm_mode']
            logging.info('  Setting ASIC pixel mode to {} '.format(csmspm_mode.value))
            write_params.append(ExcaliburParameter('mpx3_csmspmmode', [[csmspm_mode.index]]))

            disc_csm_spm = self._param['config/disc_csm_spm']
            logging.info('  Setting ASIC discriminator output mode to {} '.format(disc_csm_spm.value))
            write_params.append(ExcaliburParameter('mpx3_disccsmspm', [[disc_csm_spm.index]]))

            equalization_mode = self._param['config/equalization_mode']
            logging.info('  Setting ASIC equalization mode to {} '.format(equalization_mode.value))
            write_params.append(ExcaliburParameter('mpx3_equalizationmode', [[equalization_mode.index]]))

            gain_mode = self._param['config/gain_mode']
            logging.info('  Setting ASIC gain mode to {} '.format(gain_mode.value))
            write_params.append(ExcaliburParameter('mpx3_gainmode', [[gain_mode.index]]))

            counter_select = self._param['config/counter_select'].value
            logging.info('  Setting ASIC counter select to {} '.format(counter_select))
            write_params.append(ExcaliburParameter('mpx3_counterselect', [[counter_select]]))

            counter_depth = self._param['config/counter_depth'].value
            logging.info('  Setting ASIC counter depth to {} bits'.format(counter_depth))
            write_params.append(ExcaliburParameter('mpx3_counterdepth',
                                                   [[ExcaliburDefinitions.FEM_COUNTER_DEPTH_MAP[counter_depth]]]))

            logging.info('  Setting operation mode to {}'.format(operation_mode.value))
            write_params.append(ExcaliburParameter('mpx3_operationmode', [[operation_mode.index]]))

            lfsr_bypass = self._param['config/lfsr_bypass']
            logging.info('  Setting LFSR bypass mode to {}'.format(lfsr_bypass.value))
            write_params.append(ExcaliburParameter('mpx3_lfsrbypass', [[lfsr_bypass.index]]))

            #
            # if self.args.matrixread:
            #     lfsr_bypass_mode = ExcaliburDefinitions.FEM_LFSR_BYPASS_MODE_ENABLED
            # else:
            #     lfsr_bypass_mode = ExcaliburDefinitions.FEM_LFSR_BYPASS_MODE_DISABLED
            #
            #logging.info('  Setting data interface address and port parameters')
            #write_params.append(ExcaliburParameter('source_data_addr', [[addr] for addr in self.source_data_addr]))
            #write_params.append(ExcaliburParameter('source_data_mac', [[mac] for mac in self.source_data_mac]))
            #write_params.append(ExcaliburParameter('source_data_port', [[port] for port in self.source_data_port]))
            #write_params.append(ExcaliburParameter('dest_data_addr', [[addr] for addr in self.dest_data_addr]))
            #write_params.append(ExcaliburParameter('dest_data_mac', [[mac] for mac in self.dest_data_mac]))
            #write_params.append(ExcaliburParameter('dest_data_port', [[port] for port in self.dest_data_port]))

            logging.info('  Disabling local data receiver thread')
            write_params.append(ExcaliburParameter('datareceiver_enable', [[0]]))

            # Connect to the hardware
            # self.connect({'state': True})

            # Write all the parameters to system
            logging.info('Writing configuration parameters to system {}'.format(str(write_params)))
            self.hl_write_params(write_params)

            self._frame_start_count = 0
            self._frame_count_time = None

            # Send start acquisition command
            logging.info('Sending start acquisition command')
            self.do_command('start_acquisition', params=None)
            # if not cmd_ok:
            #     logging.error('start_acquisition command failed: {}'.format(self.client.error_msg))
            #     return
            #
            # # If the nowait arguments wasn't given, monitor the acquisition state until all requested frames
            # # have been read out by the system
            # if not self.args.no_wait:
            #
            #     wait_count = 0
            #     acq_completion_state_mask = 0x40000000
            #     frames_acquired = 0
            #
            #     while True:
            #
            #         (read_ok, vals) = self.client.fe_param_read(['frames_acquired', 'control_state'])
            #         frames_acquired = min(vals['frames_acquired'])
            #         acq_completed = all(
            #             [((state & acq_completion_state_mask) == acq_completion_state_mask) for state in vals['control_state']]
            #         )
            #         if acq_completed:
            #             break
            #
            #         wait_count += 1
            #         if wait_count % 5 == 0:
            #             logging.info('  {:d} frames read out  ...'.format(frames_acquired))
            #
            #     logging.info('Completed readout of {} frames'.format(frames_acquired))
            #     self.do_stop()
            #     logging.info('Acquisition complete')
            # else:
            #     logging.info('Acquisition started, not waiting for completion, will not send stop command')

#    def read_ini_file(self):

    def hl_initialise(self):
        self.do_command('fe_init', params=None)
        return self.wait_for_completion()

    def hl_lv_enable(self, lv_enable):

        if self.powercard_fem_idx <= 0:
            self.set_error("Unable to set LV enable as server reports no power card")
            return
        params = []
        params.append(ExcaliburParameter('fe_lv_enable', [[lv_enable]], fem=self.powercard_fem_idx+1))
        self.hl_write_params(params)

    def hl_hv_enable(self, hv_enable):

        if self.powercard_fem_idx <= 0:
            self.set_error("Unable to set HV enable as server reports no power card")
            return
        params = []
        params.append(ExcaliburParameter('fe_hv_enable', [[hv_enable]], fem=self.powercard_fem_idx+1))
        self.hl_write_params(params)

    def hl_hv_bias_set(self, name, value):
        if self.powercard_fem_idx <= 0:
            self.set_error("Unable to set HV bias [] as server reports no power card".format(name))
            return

        params = []
        params.append(ExcaliburParameter('fe_hv_bias', [[value]], fem=self.powercard_fem_idx+1))
        self.hl_write_params(params)

    def hl_stop_acquisition(self):
        with self._comms_lock:
            self.do_command('stop_acquisition', None)
            return self.wait_for_completion()

    def hl_write_params(self, params):
        with self._comms_lock:
            self.write_fe_param(params)
            return self.wait_for_completion()

    def get_fem_error_state(self):
        fem_state = self.get('status/fem')['fem']
        logging.debug("%s", fem_state)
        for (idx, state) in enumerate(fem_state):
            yield (idx, state['id'], state['error_code'], state['error_msg'])

    def wait_for_completion(self):
        succeeded = False
        err_msg = ''
        try:
            while True:
                time.sleep(0.1)
                if not self.get('status/command_pending')['command_pending']:
                    succeeded = self.get('status/command_succeeded')['command_succeeded']
                    if succeeded:
                        pass
                    else:
                        logging.error('Command write_fe_param failed on following FEMS:')
                        fem_error_count = 0
                        for (idx, fem_id, error_code, error_msg) in self.get_fem_error_state():
                            if error_code != 0:
                                logging.error(
                                    '  FEM idx {} id {} : {} : {}'.format(idx, fem_id, error_code, error_msg))
                                fem_error_count += 1
                        err_msg = 'Command write_fe_param failed on {} FEMs'.format(fem_error_count)
                    break

        except ExcaliburDetectorError as e:
            err_msg = str(e)

        if not succeeded:
            self.set_error(err_msg)

        return succeeded, err_msg
