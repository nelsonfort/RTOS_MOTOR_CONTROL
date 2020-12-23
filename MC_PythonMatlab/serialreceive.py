from serial import Serial
import struct
import signal
import sys
import csv



def handler(signal, frame):
    print('welcome to the handler')

    ser.close()
    csvfile.close()
    sys.exit(0)

signal.signal(signal.SIGINT, handler)

ser = Serial ("COM4",115200 )    #Open port with baud rate

csvfile = open('speedVStimePI2.csv', 'w', newline='')
fieldnames = ['time', 'speed','ref']
writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

writer.writeheader()

while 1:
    #Reseteo los flags de comienzo de trama de sincronización
    value_a = -1
    value_b = -1
    value_c = -1
    value_d = -1

    processing = True
    while processing:
        check_begin = ser.read(1)

        value_d, = struct.unpack('<B', check_begin)

        if((value_a == 8) and (value_b == 16) and (value_c == 32) and (value_d == 64)):
            data = ser.read(5*4) #Two unsigned ints of 32 bits
            (data_speed, data_period, data_freq, data_time, reference) = struct.unpack('<fIfIf', data)
         
            print("-- Speed: ", data_speed,"-- time: ",data_time,"-- ref: ",reference)

            writer.writerow({'time': data_time/1000, 'speed': data_speed*60, 'ref': reference})
            value_a = -1
            value_b = -1
            value_c = -1
            value_d = -1
        else:
            value_a = value_b
            value_b = value_c
            value_c = value_d
        