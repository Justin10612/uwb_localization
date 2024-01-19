import serial

baudRate = 112500

# Serial Port
# serial_arduino = serial.Serial('COM1', baudRate, timeout=1)
serial_uwb0 = serial.Serial('/dev/ttyACM1', baudRate, timeout=1)
serial_uwb1 = serial.Serial('/dev/ttyACM2', baudRate, timeout=1)
serial_uwb2 = serial.Serial('/dev/ttyACM0', baudRate, timeout=1)
distList = []

def send_and_receive(uwbNum, serial_receive):
    data = serial_receive.readline()
    dis = data.decode('utf-8').strip()
    k=0
    while(dis==''):
        data = serial_receive.readline()
        dis = data.decode('utf-8').strip()
        # print('dis=' + str(dis))
        print('No Data, Wating:'+str(uwbNum))
        pass
    distList.append(dis)
    
try:
    while True:
        distList = []
        i=0
        send_and_receive(i, serial_uwb0)
        i=1
        send_and_receive(i, serial_uwb1)
        i=2
        send_and_receive(i, serial_uwb2)
        print(distList)
except KeyboardInterrupt:
    serial_uwb0.close()
    serial_uwb1.close()
    serial_uwb2.close()
    print("Program terminated by user.") 