import numpy as np
import Espros_API_Support_Functions as support
import cmd
import socket
import cv2
import struct

REMOTE_IP = '192.168.6.100'
buffer = 8192
PORT = 50660


class Espros_Commands(cmd.Cmd):
    intro = "Welcome to Espros ToF camera CLI (use help for list of commands)"
    prompt = "_ESPROS_TOF_CAM_: "
    ruler = ""  # makes help look a bit better

    def do_getAmplitudeSorted(s, parms):
        # send command
        s.sendall(bytes('getAmplitudeSorted\n'.encode('ascii')))

        # get answer
        coded_data = []
        data = []
        while data != b'':
            data = s.recv(buffer)
            coded_data.append(data)

        if parms == 'frame':
            print('Mcdobe Sandstorm')
        elif parms == 'raw':
            # process and return data
            amp = support.decodeImg(coded_data)
            img = amp.copy().astype('uint16')
            return img
        else:
            return coded_data

    def do_getDistanceSorted(s, parms):
        "Get a distance image from the ToF camera"
        # send command
        s.sendall(bytes('getDistanceSorted\n'.encode('ascii')))

        # get answer
        coded_data = []
        data = []

        while data != b'':
            data = s.recv(buffer)
            coded_data.append(data)

        if parms == 'frame':
            print('Mcdobe Sandstorm')
        if parms == 'raw':
            # process and return data
            dist = support.decodeImg(coded_data)
            img = dist.copy().astype('uint16')
            return img
        else:
            return coded_data

    def do_getDCSSorted(s, parms):
        "Gets DCS data from the camera"

        # send command
        s.sendall(bytes('getDCSSorted\n'.encode('ascii')))

        # get answer
        coded_data = []
        data = []

        while data != b'':
            data = s.recv(buffer)
            coded_data.append(data)

        if parms == 'frame':
            print('mcdobe sandstorm')
        elif parms == 'raw':

            # process and return data as 32-bit little endian
            decoded_data = support.decodeDCSImg(coded_data)

            # casting to a 16-bit unsigned
            dcs0 = decoded_data[0, :, :]
            dcs1 = decoded_data[1, :, :]
            dcs2 = decoded_data[2, :, :]
            dcs3 = decoded_data[3, :, :]

            # clears the 4 most significant little endian bits and subtracts 2048 (from Espros data sheet)
            dcs0 = np.bitwise_and(dcs0, 0x00000fff).astype(np.int16) - 2048
            dcs1 = np.bitwise_and(dcs1, 0x00000fff).astype(np.int16) - 2048
            dcs2 = np.bitwise_and(dcs2, 0x00000fff).astype(np.int16) - 2048
            dcs3 = np.bitwise_and(dcs3, 0x00000fff).astype(np.int16) - 2048
            return dcs0, dcs1, dcs2, dcs3
        else:
            return coded_data

    def do_setROI(s, parms):
        # TODO: make function
        return

    def do_setIntegrationTime2D(s, parms):
        # TODO: make function
        return

    def do_setIntegrationTime3D(s, parms):
        "Sets integration time of the camera || 1 argument: time in micro seconds (max is 4000)\nEx: setIntegrationTime3D 1000"
        int_time = arg_to_argv(parms)
        if int(int_time[0]) <= 4000:
            print("Integration time set to", int_time[0])
            s = build_tcp_connection()
            s.sendall(bytes(('setIntegrationTime3D ' + str(int_time[0]) + '\n').encode('ascii')))
        elif int(int_time[0]) > 315000:
            print("Integration time set to 419 ms (highest possible)")
            writeIntegrationRegisters(s, '02', '00', 'FF', 'FB')
        elif int(int_time[0]) > 157000:
            print("Integration time set to 209 ms")
            writeIntegrationRegisters(s, '01', '00', 'FF', 'FB')
        elif int(int_time[0]) > 78000:
            print("Integration time set to 105 ms")
            writeIntegrationRegisters(s, '00', '80', 'FF', 'FB')
        elif int(int_time[0]) > 39000:
            print("Integration time set to 52 ms")
            writeIntegrationRegisters(s, '00', '40', 'FF', 'FB')
        elif int(int_time[0]) > 19500:
            print("Integration time set to 26 ms")
            writeIntegrationRegisters(s, '00', '20', 'FF', 'FB')
        elif int(int_time[0]) > 9800:
            print("Integration time set to 13.1 ms")
            writeIntegrationRegisters(s, '00', '10', 'FF', 'FB')
        elif int(int_time[0]) > 4000:
            print("Integration time set to 6553 us")
            writeIntegrationRegisters(s, '00', '08', 'FF', 'FB')
        return

    def do_setIntegrationTime3DHDR(s, parms):
        # TODO: make function
        return

    def do_setABS(s, parms):
        # TODO: make function
        return

    def do_selectMode(s, parms):
        # TODO: make function
        return

    def do_selectPolynomial(s, parms):
        # TODO: make function
        return

    def do_loadConfig(s, parms):
        # TODO: make function
        return

    def do_readRegister(s, register):

        msg = 'readRegister ' + str(register) + '\n'
        s.sendall(bytes(msg.encode('ascii')))
        reply = s.recv(buffer)
        reply = reply[0:1]
        print("Register:" + str(register) + " Value:" + str(reply))
        return reply

    def do_writeRegister(s, parms):
        # TODO: make function
        regToRead = arg_to_argv(parms)
        s = build_tcp_connection()
        s.sendall(bytes(('w ' + str(regToRead[0]) + ' ' + str(regToRead[1]) + '\n').encode('ascii')))
        return

    def do_enableImaging(s, parms):
        # TODO: make function
        return

    def do_getOffset(s, parms):
        # TODO: make function
        return

    def do_getBadPixels(s, parms):
        # TODO: make function
        return

    def do_getTemperature(s):
        "returns an an array of six temperature readings on the Espros unit in °C"
        # order is [top left (chip), top right (chip), bottom left (chip), bottom right (chip), ADC4 (LED top), ADC2 (LED bottom)]
        s.sendall(bytes(('getTemperature\n').encode('ascii'))) # this api call takes no arguments
        temperatureArray = s.recv(buffer)
        return np.frombuffer(temperatureArray, np.dtype('uint16'), -1) / 10 # cast bytestring to uint16, then convert from deci-degrees C to deg C

    def do_getAveragedTemperature(s):
        "returns the averaged chip temperature (tl, tr, bl, br) in °C"
        # Note that this command requires the Beaglebone to do extra computation after it calls `getTemperatures` and then averages the first four (chip) temps
        s.sendall(bytes(('getAveragedTemperature\n').encode('ascii'))) # This api call takes no arguments
        reply = s.recv(buffer)
        return int.from_bytes(reply, byteorder='little') / 40 # convert uint16 to integer, then divide by (4 temps * 10 deciDegrees/degree)

    def do_getChipInfo(s, parms):
        # TODO: make function
        return

    def do_getIcVersion(s, parms):
        # TODO: make function
        return

    def do_getPartVersion(s, parms):
        # TODO: make function
        return

    def do_version(s, parms):
        # TODO: make function
        return

    def do_getSpeedOfLight(s, parms):
        # TODO: make function
        return

    def do_getBWSorted(s, parms):
        # TODO: make function
        return

    def do_getModulationFrequencies(s):
        msg = "getModulationFrequencies\n"
        s.sendall(bytes(msg.encode('ascii')))
        reply = s.recv(buffer)
        reply_as_array = np.frombuffer(reply, np.dtype('uint16'), -1)
        return reply_as_array[1::2]  # only return odd-indexed elements; rest are zeros

    def do_enableIllumination(s, parms):
        msg = 'enableIllumination' + str(parms) + '\n'
        s.sendall(bytes(msg.encode('ascii')))
        return

    def do_enableVerticalBinning(s, parms):
        # TODO: make function
        return

    def do_enableHorizontalBinning(s, parms):
        # TODO: make function
        return

    def do_setRowReduction(s, parms):
        # TODO: make function
        return

    def do_filterMode(s, parms):
        # TODO: make function
        return

    def do_nLoopFilter(s, parms):
        # TODO: make function
        return

    def do_enableArgCheck(s, parms):
        # TODO: make function
        return

    def do_setAddArgThreshold(s, parms):
        # TODO: make function
        return

    def do_setAddArgMin(s, parms):
        # TODO: make function
        return

    def do_setAddArgMax(s, parms):
        # TODO: make function
        return

    def do_setMinAmplitude(s, parms):
        # TODO: make function
        return

    def do_getMinAmp(s, parms):
        # TODO: make function
        return

    def do_enableDualMGX(s, parms):
        # TODO: make function
        return

    def do_enableHDR(s, parms):
        # TODO: make function
        return

    def do_enablePiDelay(s, parms):
        # TODO: make function
        return

    def do_setModulationFrequency(s, parms):
        msg = 'setModulationFrequency ' + str(parms) + '\n'
        s.sendall(bytes(msg.encode('ascii')))
        return

    def do_getCalibrationTypeForFreqIdx(s, parms):
        # returns calibration type (or None)
        # input: an integer [0,7] corresponding to index of modulation frequency
        msg = "getCalibrationTypeForFreqIdx " + str(parms).strip() + '\n'
        s.sendall(bytes(msg.encode('ascii')))
        reply = s.recv(buffer)
        return int.from_bytes(reply, byteorder='little')

    def do_selectMode(s, parms):
        # TODO: make function
        return

    def do_EnableTemperatureCorrection(s, parms):
        "sets a flag within the Beaglebone code that applies temperature correction to distance and amplitude measurements"
        # note: the boolean flag to enable temperature correction will be overridden to False if no distance
        # calibration has been performed if parms='1', enable; if parms='0', disable calling this api command always
        # returns int 0 (will ignore)
        s.sendall(bytes(('correctTemperature ' + str(parms).strip() + '\n').encode('ascii')))

    def do_getIsTemperatureCorrectionEnabled(s):
        "note: the boolean flag to enable temperature correction will be overridden to False if no distance calibration has been performed"
        s.sendall(bytes(('isTemperatureCorrectionEnabled\n').encode('ascii')))  # this api call takes no arguments
        reply = s.recv(buffer)
        reply_as_int = int.from_bytes(reply, byteorder='little')
        if reply_as_int == 1:
            return True
        else:
            return False

    def do_exit(self, arg):
        "Exit the program"
        return True


def build_tcp_connection():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)

    try:
        s.connect((REMOTE_IP, PORT))
    except:
        print()
    return s


def arg_to_argv(arg):
    return arg.split()


def writeRegister(s, register, value):
    # TODO: make function
    s = build_tcp_connection()
    s.sendall(bytes(('w ' + str(register) + ' ' + str(value) + '\n').encode('ascii')))
    return


def writeIntegrationRegisters(s, a0, a1, a2, a3):
    writeRegister(s, 'A0', a0)
    writeRegister(s, 'A1', a1)
    writeRegister(s, 'A2', a2)
    writeRegister(s, 'A3', a3)
    return
