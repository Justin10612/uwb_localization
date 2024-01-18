import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Vector3
from std_msgs.msg import String


class UWBInit(Node):

    baudRate = 112500

    # Serial Port
    # serial_arduino = serial.Serial('COM1', baudRate, timeout=1)
    serial_uwb0 = serial.Serial('ttyACM0', baudRate, timeout=1)
    serial_uwb1 = serial.Serial('ttyACM2', baudRate, timeout=1)
    serial_uwb2 = serial.Serial('ttyACM1', baudRate, timeout=1)
    distList = []

    def __init__(self):
        super().__init__('uwb_init')
        # Subscriber
        self.mode_sub_ = self.create_subscription(String, 'robot_mode', self.mode_callback, 10)
        self.mode_sub_
        # Publisher
        self.pose_pub_ = self.create_publisher(Vector3, 'uwb_distance', 10)
        # Serial Port
        serial.Serial('ttyACM0', 115200, timeout=1)
        
    def mode_callback(self, msg):
        # Variable
        # Receive Mode
        robot_mode = msg.data
        # Receieve Dis Data
        dis_msgs = Vector3()
        try:
            while True:
                self.distList = []
                i=0
                self.send_and_receive(i, self.serial_uwb0)
                dis_msgs.x = self.distList[i]
                i=1
                self.send_and_receive(i, self.serial_uwb1)
                dis_msgs.y = self.distList[i]
                i=2
                self.send_and_receive(i, self.serial_uwb2)
                dis_msgs.z = self.distList[i]
                self.pose_pub_.publish(dis_msgs)
        except KeyboardInterrupt:
            self.serial_uwb0.close()
            self.serial_uwb1.close()
            self.serial_uwb2.close()     
        
                

    def send_and_receive(self, uwbNum, serial_receive):
        data = serial_receive.readline()
        dis = data.decode('utf-8').strip()
        k=0
        while(dis==''):
            data = serial_receive.readline()
            dis = data.decode('utf-8').strip()
            print('dis=' + str(dis))
            print('Wating:'+str(uwbNum))
            pass
        self.distList.append(dis)
        

def main(args=None):
    rclpy.init(args=args)
    uwb_init = UWBInit()
    rclpy.spin(uwb_init)
    uwb_init.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()