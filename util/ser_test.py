#!/usr/bin/python3

import sys
import serial
from time import perf_counter, sleep
import threading


def format_bytes(size):
   B = float(size)
   KB = float(1024)
   MB = float(KB ** 2) # 1,048,576
   if B < KB:
      return '{0} {1}'.format(B,'B')
   elif KB <= B < MB:
      return '{0:.2f} KB'.format(B/KB)
   else :
      return '{0:.2f} MB'.format(B/MB)

def rx_thread(ser, datalen):
    global rx_data
    rx_data = ser.read(datalen)

def main():
    if len(sys.argv) == 1:
        print("No port name specified")
        exit()
    
    ser = serial.Serial(*sys.argv[1:])
    
    data = bytes()
    for i in range(4096):
        data = data + bytes(range(256))
    
    thr = threading.Thread(target=rx_thread, args=(ser, len(data)))
    thr.start()

    print("Testing port", sys.argv[1])
    
    t_start = perf_counter()
    ser.write(data)
    t_stop = perf_counter()
    print("Throughput:", format_bytes(len(data) / (t_stop - t_start)) + "/s")

    thr.join(timeout=1.0)
    
    global rx_data
    
    # f = open("data.txt", "wb")
    # f.write(data)
    # f.close()
    
    # f = open("rx_data.txt", "wb")
    # f.write(rx_data)
    # f.close()
    
    if thr.is_alive():
        print("rx timeout")
        ser.cancel_read()
    else:
        if rx_data == data:
            print("rx ok")
        else:
            print("rx corrupted")

if __name__ == "__main__":
    main()