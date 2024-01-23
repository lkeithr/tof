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

    def do_getTemperature(s, parms):
        # TODO: make function
        return

    def do_getAveragedTemperature(s, parms):
        # TODO: make function
        return

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

    def do_getModulationFrequencies(s, parms):
        # TODO: make function
        return

    def do_getCalibrationTypeForFreqIdx(s, parms):
        # TODO: make function
        return

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

    def do_selectMode(s, parms):
        # TODO: make function
        return

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
