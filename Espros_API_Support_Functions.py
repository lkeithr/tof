import numpy as np


def decodeImg(coded_data):
    msg = b''
    total = 0
    for data_msg in coded_data:
        msg += data_msg
        data_len = len(data_msg)
        total += data_len

    npArray = np.frombuffer(msg, np.dtype('<u2'), int(total / 2))
    return np.reshape(npArray, (240, 320), order='C') if npArray.size > 0 else -1


def decodeDCSImg(coded_data):
    msg = b''
    total = 0
    for data_msg in coded_data:
        msg += data_msg
        data_len = len(data_msg)
        total += data_len

    npArray = np.frombuffer(msg, np.dtype('<u2'), int(total / 2))
    return np.reshape(npArray, (4, 240, 320), order='C') if npArray.size > 0 else -1