import rclpy
from rclpy.node import Node
import serial

from geometry_msgs.msg import Vector3
from std_msgs.msg import String


class UWBReceiver(Node):

    baudRate = 115200
    distList = []
    #
    last_0 = 0.0
    last_1 = 0.0
    last_2 = 0.0  
    # Serial Port
    serial_uwb0 = serial.Serial('/dev/ttyACM0', baudRate, timeout=1)
    serial_uwb1 = serial.Serial('/dev/ttyACM1', baudRate, timeout=1)
    serial_uwb2 = serial.Serial('/dev/ttyACM2', baudRate, timeout=1)
    
    def __init__(self):
        super().__init__('uwb_init')
        # Subscriber
        self.mode_sub_ = self.create_subscription(String, 'robot_mode', self.mode_callback, 10)
        self.mode_sub_
        # Publisher
        self.pose_pub_ = self.create_publisher(Vector3, 'uwb_distance', 10)
        # Reset Serial
        self.serial_uwb0.reset_input_buffer()
        self.serial_uwb1.reset_input_buffer()
        self.serial_uwb2.reset_input_buffer()

    def mode_callback(self, msg):
        # Receieve Dis Data
        dis_msgs = Vector3()
        self.get_logger().info('UWB_dist Start')
        try:
            while True:
                i=0
                dis_anchor0 = send_and_receive(i, self.serial_uwb0)
                if dis_anchor0<= 0:
                    dis_msgs.x = self.last_0
                else:
                    dis_msgs.x = dis_anchor0
                    self.last_0 = dis_anchor0
                i=1
                dis_anchor1 = send_and_receive(i, self.serial_uwb1)
                if dis_anchor1<= 0:
                    dis_msgs.y = self.last_1
                else:
                    dis_msgs.y = dis_anchor1
                    self.last_1 = dis_anchor1
                i=2
                dis_anchor2 = send_and_receive(i, self.serial_uwb2)
                if dis_anchor2 <= 0:
                    dis_msgs.z = self.last_2
                else:
                    dis_msgs.z = dis_anchor2
                    self.last_2 = dis_anchor2
                self.pose_pub_.publish(dis_msgs)
        except KeyboardInterrupt:
            self.serial_uwb0.close()
            self.serial_uwb1.close()
            self.serial_uwb2.close()
            print("Program terminated by user.")

def send_and_receive(uwbNum, serial_receive):
    data = serial_receive.readline()
    dis = data.decode('utf-8').strip()
    while(dis==''):
        data = serial_receive.readline()
        dis = data.decode('utf-8').strip()
        # print('dis=' + str(dis))
        # print('No Data, Wating:'+str(uwbNum))
    return float(dis)

def main(args=None):
    rclpy.init(args=args)
    uwb_init = UWBReceiver()
    rclpy.spin(uwb_init)
    uwb_init.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()