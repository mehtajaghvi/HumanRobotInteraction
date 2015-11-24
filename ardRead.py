import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
while True:
     print ser.readline()
'1 Hello world!\r\n'
'2 Hello world!\r\n'
'3 Hello world!\r\n'
