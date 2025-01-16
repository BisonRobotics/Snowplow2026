import numpy as np
from scipy.signal import argrelmin

def find_minima(scan_msg):
    # tup is (dist, ang)

    # list of tup
    sorted_scan_msg = np.array(sorted(scan_msg, key=lambda tup: tup[1]))

    window_size = 5

    arr = np.convolve(sorted_scan_msg[:, 0], np.ones(window_size), 'valid') / window_size

    # arr is list of dist
    # arr = sorted_scan_msg[:, 0]

    return sorted_scan_msg[argrelmin(arr)]