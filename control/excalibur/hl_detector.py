"""
interface_wrapper.py - EXCALIBUR high level API for the ODIN server.

Alan Greer, DLS
"""
import logging
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
        self.source_data_addr = [u'10.0.2.2']
        self.source_data_mac = [u'62:00:00:00:00:01']
        self.source_data_port = [8]
        self.dest_data_addr = [u'10.0.2.1']
        self.dest_data_mac = [u'00:07:43:06:31:A7']
        self.dest_data_port = [61649]
        # TODO: Hardcoded 2 fems, this needs updating
        self._fems = [1, 2]

        # Create the calibration object
        self._cb = DetectorCalibration()

        # Create the Excalibur parameters
        self._param = {
            'config/num_images': IntegerParameter('num_images', 1),
            'config/acquisition_time': IntegerParameter('acquisition_time', 1000),
            'config/num_test_pulses': IntegerParameter('num_test_pulses', 0),
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
            'config/energy_threshold': StringParameter('energy_threshold', 0.0, callback=self.update_calibration)

            #["Normal",
            #                                                                     "Burst",
            #                                                                     "Histogram",
            #                                                                     "DAC Scan",
            #                                                                     "Matrix Read"])
        }
        self._status = {}
        self._update_time = datetime.now()
        self._status_thread = threading.Thread(target=self.status_loop)
        #self._status_thread.start()

    def update_calibration(self, name, value):
        logging.debug("Updating calibration due to %s updated to %s", name, value)
        self._cb.set_file_root(self._param['config/cal_file_root'].value)
        self._cb.set_csm_spm_mode(self._param['config/csm_spm_mode'].index)
        self._cb.set_gain_mode(self._param['config/gain_mode'].index)
        self._cb.set_energy_threshold(self._param['config/energy_threshold'].value)
        self._cb.load_calibration_files(self._fems)
        self.download_dac_calibration()
        self.download_pixel_calibration()

    def download_dac_calibration(self):
        dac_params = []
        chip_ids = [1, 2, 3, 4, 5, 6, 7, 8]

        #            logging.debug("%s", self._cb.get_dac(1))
        for (dac_name, dac_param) in self._cb.get_dac(1).dac_api_params():
            logging.debug("%s  %s", dac_name, dac_param)
#
            dac_vals = []
#            for (fem_id, fem_idx) in zip(fem_ids, fem_idxs):
            for fem in self._fems:
    #

                fem_vals = [self._cb.get_dac(fem).dacs(fem, chip_id)[dac_name] for chip_id in chip_ids]
                dac_vals.append(fem_vals)

            dac_params.append(ExcaliburParameter(dac_param, dac_vals,
                                                 fem=self._fems, chip=chip_ids))

#        dac_params.append(ExcaliburParameter('mpx3_dacsense', [[self.args.sense_dac]],
#                                             fem=self.args.config_fem, chip=self.args.config_chip))
#
        # Connect to the hardware
        self.connect({'state': True})

        time.sleep(1.0)

        # Write all the parameters to system
        logging.info('Writing DAC configuration parameters to system {}'.format(str(dac_params)))
        self.write_fe_param(dac_params)

    #        write_ok = self.client.fe_param_write(dac_params)
#        if not write_ok:
#            logging.error('Failed to write DAC parameters for FEM ID {}, chip ID {}'.format(fem_id, chip_id))
#            return
#
#        load_ok = self.client.do_command('load_dacconfig', self.args.config_fem, self.args.config_chip)
#        if load_ok:
#            logging.info('DAC load completed OK')
#        else:
#            logging.error('Failed to execute DAC load command: {}'.format(self.client.error_msg))

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

    def status_loop(self):
        while True:
            if (datetime.now() - self._update_time).seconds > 5.0:
                self._update_time = datetime.now()
                self.slow_read()

    def get(self, path):
        if path in self._param:
            response = self._param[path].get()
        elif self.search_status(path) is not None:
            response = {'value': self.search_status(path)}
        else:
            response = super(HLExcaliburDetector, self).get(path)

        return response

    def set(self, path, data):
        if path in self._param:
            self._param[path].set_value(data)
        elif path == 'command/start_acquisition':
            # Starting an acquisition!
            logging.debug('Start acquisition has been called')
            self.do_acquisition()
        else:
            super(HLExcaliburDetector, self).set(path, data)

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

    # def
    #
    #     state = response.json()['status']
    #
    #     if not state['command_pending']:
    #         succeeded = state['command_succeeded']
    #         if succeeded:
    #             response = requests.get(self.url + 'command/' + cmd)
    #         else:
    #             self.logger.error('Command {} failed on following FEMS:'.format(cmd))
    #             fem_error_count = 0
    #             for (idx, fem_id, error_code, error_msg) in self.get_fem_error_state():
    #                 if error_code != 0:
    #                     self.logger.error('  FEM idx {} id {} : {} : {}'.format(idx, fem_id, error_code, error_msg))
    #                     fem_error_count += 1
    #             self.error_code = ExcaliburDefinitions.ERROR_FEM
    #             self.error_msg = 'Command {} failed on {} FEMs'.format(cmd, fem_error_count)
    #         break

    def slow_read(self):

        # Connect to the hardware
        if not self.connected:
            self.connect({'state': True})

        # Wait
        time.sleep(1.0)

        fem_params = ['fem_local_temp', 'fem_remote_temp', 'moly_temp', 'moly_humidity']
        supply_params = ['supply_p1v5_avdd1', 'supply_p1v5_avdd2', 'supply_p1v5_avdd3', 'supply_p1v5_avdd4',
                        'supply_p1v5_vdd1', 'supply_p2v5_dvdd1']

        fe_params = fem_params + supply_params + ['mpx3_dac_out']

        read_params = ExcaliburReadParameter(fe_params)
        self.read_fe_param(read_params)

        while True:
            logging.info("Waiting....")
            time.sleep(0.1)
            if not self.command_pending():
                if self._get('command_succeeded'):
                    logging.info("Command has succeeded")
                else:
                    logging.info("Command has failed")
                break
        self._status.update(super(HLExcaliburDetector, self).get('command')['command']['fe_param_read']['value'])
        logging.info("Results: %s", self._status)



    # def exec_command(self, cmd, params=None):
    #
    #     self.logger.debug('Executing command: {}'.format(cmd))
    #
    #     payload = {cmd: params}
    #     response = requests.put(self.url + 'command', data=json.dumps(payload), headers=self.request_headers)
    #
    #     succeeded = False
    #
    #     if response.status_code == requests.codes.ok:
    #
    #         while True:
    #             time.sleep(self.cmd_completion_poll_interval)
    #             response = requests.get(self.url + 'status')
    #             state = response.json()['status']
    #
    #             if not ######state['command_pending']:
    #                 succeeded = state['command_succeeded']
    #                 if succeeded:
    #                     response = requests.get(self.url + 'command/' + cmd)
    #                 else:
    #                     self.logger.error('Command {} failed on following FEMS:'.format(cmd))
    #                     fem_error_count = 0
    #                     for (idx, fem_id, error_code, error_msg) in self.get_fem_error_state():
    #                         if error_code != 0:
    #                             self.logger.error(
    #                                 '  FEM idx {} id {} : {} : {}'.format(idx, fem_id, error_code, error_msg))
    #                             fem_error_count += 1
    #                     self.error_code = ExcaliburDefinitions.ERROR_FEM
    #                     self.error_msg = 'Command {} failed on {} FEMs'.format(cmd, fem_error_count)
    #                 break
    #
    #     else:
    #         self.error_code = ExcaliburDefinitions.ERROR_REQUEST
    #         if 'error' in response.json():
    #             self.error_msg = response.json()['error']
    #         else:
    #             self.error_msg = 'unknown error'
    #
    #         self.error_msg = 'Command {} request failed with code {}: {}'.format(
    #             cmd, response.status_code, self.error_msg)
    #
    #         self.logger.error(self.error_msg)
    #
    #     return (succeeded, response.json())

    def do_acquisition(self):
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
        tp_enable = 1 if tp_count != 0 else 0
        write_params.append(ExcaliburParameter('testpulse_enable', [[tp_enable]]))

        num_frames = self._param['config/num_images'].value
        logging.info('  Setting number of frames to {}'.format(num_frames))
        write_params.append(ExcaliburParameter('num_frames_to_acquire', [[num_frames]]))

        acquisition_time = self._param['config/acquisition_time'].value
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
        logging.info('  Setting data interface address and port parameters')
        write_params.append(ExcaliburParameter('source_data_addr', [[addr] for addr in self.source_data_addr]))
        write_params.append(ExcaliburParameter('source_data_mac', [[mac] for mac in self.source_data_mac]))
        write_params.append(ExcaliburParameter('source_data_port', [[port] for port in self.source_data_port]))
        write_params.append(ExcaliburParameter('dest_data_addr', [[addr] for addr in self.dest_data_addr]))
        write_params.append(ExcaliburParameter('dest_data_mac', [[mac] for mac in self.dest_data_mac]))
        write_params.append(ExcaliburParameter('dest_data_port', [[port] for port in self.dest_data_port]))

        logging.info('  Disabling local data receiver thread')
        write_params.append(ExcaliburParameter('datareceiver_enable', [[0]]))

        # Connect to the hardware
        self.connect({'state': True})

        # Write all the parameters to system
        logging.info('Writing configuration parameters to system {}'.format(str(write_params)))
        self.write_fe_param(write_params)

        # Send start acquisition command
        logging.info('Sending start acquisition command')
        # cmd_ok = self.client.do_command('start_acquisition')
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