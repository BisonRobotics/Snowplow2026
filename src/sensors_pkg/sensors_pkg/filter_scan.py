def filter_polar(scan_msg):
    # tup[0] is x
    # tup[1] is y
    return list(filter(lambda tup: -8 <= tup[0] <= 8 and 0 <= tup[1] <= 4, scan_msg))