import rclpy
from rclpy.node import Node
import serial
import numpy as np
import time

from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class custom_kalman1D:
  def __init__(self, Q, R):
    n = 10
    # self.Q = 1e-5  # 過程噪聲協方差
    # self.R = 0.01**2  # 觀測噪聲協方差
    self.Q = Q  # 過程噪聲協方差
    self.R = R  # 觀測噪聲協方差
    self.xhat = np.zeros(n)  # 估計值
    self.xhat = [0,0]
    self.P = np.zeros(n)  # 估計值的協方差
    self.P =[0]
    self.xhatminus = np.zeros(n)  # 預測值
    self.xhatminus =[0]
    self.Pminus = np.zeros(n)  # 預測值的協方差
    self.Pminus =[0]
    self.K = np.zeros(n)  # 卡爾曼增益
    self.K =[0]
    self.xhat[0] = 0.0
    self.P[0] = 100.0

  def renew_and_getdata(self, raw_data):
    # 預測
    # xhatminus[k] = xhat[k-1]
    self.xhatminus[0]=self.xhat[0]
    # Pminus[k] = P[k-1] + Q
    self.Pminus[0]=self.P[0]+self.Q
    # 更新
    # K[k] = Pminus[k] / (Pminus[k] + R)
    self.K[0]=self.Pminus[0]/(self.Pminus[0]+self.R)
    # xhat[k] = xhatminus[k] + K[k] * (data[k] - xhatminus[k])
    self.xhat[1]=self.xhatminus[0]+self.K[0]*(raw_data-self.xhatminus[0])
    # P[k] = (1 - K[k]) * Pminus[k]
    self.P[0] = (1 - self.K[0]) * self.Pminus[0]
    self.xhat[0]=self.xhat[1]
    return self.xhat[0]


class UWBReceiver(Node):

  a0_kalman = custom_kalman1D(Q=1e-4, R=0.05**2)
  a1_kalman = custom_kalman1D(Q=1e-5, R=0.01**2)
  a2_kalman = custom_kalman1D(Q=1e-5, R=0.01**2)

  baudRate = 115200
  delayTime = 0.025
  d0 = []
  d1 = []
  d2 = []
  listNum = 3
  #
  last_0 = 0.0
  last_1 = 0.0
  last_2 = 0.0  
  # Serial Port
  serial_uwb0 = serial.Serial('/dev/ttyACM1', baudRate, timeout=1)
  serial_uwb1 = serial.Serial('/dev/ttyACM2', baudRate, timeout=1)
  serial_uwb2 = serial.Serial('/dev/ttyACM3', baudRate, timeout=1)
  serial_arduino = serial.Serial('/dev/ttyACM0', 230400, timeout=1)
    
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
    self.serial_arduino.reset_output_buffer()

  def mode_callback(self, msg):
    # self.serial_uwb0.reset_input_buffer()
    # self.serial_uwb1.reset_input_buffer()
    # self.serial_uwb2.reset_input_buffer()
    self.serial_arduino.reset_output_buffer()
    # Receieve Dis Data
    dis_msgs = Vector3()
    self.get_logger().info('UWB_dist Start')
    try:
      while True:
        r0 = send_and_receive(0, self.serial_uwb0)
        if r0 is not None:
            # dis_msgs.y = r0
            r0 = self.a0_kalman.renew_and_getdata(r0)
            self.serial_arduino.write(b'1\n')
            time.sleep(self.delayTime)
        dis_msgs.x = r0
        r1 = send_and_receive(1, self.serial_uwb1)
        if r1 is not None:
            r1 = self.a1_kalman.renew_and_getdata(r1)
            self.serial_arduino.write(b'2\n')
            time.sleep(self.delayTime)
        dis_msgs.y = r1
        r2 = send_and_receive(2, self.serial_uwb2)
        if r2 is not None:
            r2 = self.a2_kalman.renew_and_getdata(r2)
            self.serial_arduino.write(b'0\n')
            time.sleep(self.delayTime)
        dis_msgs.z = r2
        self.pose_pub_.publish(dis_msgs)
    except KeyboardInterrupt:
      self.serial_uwb0.close()
      self.serial_uwb1.close()
      self.serial_uwb2.close()
      print("Program terminated by user.")

def send_and_receive(uwbNum, serial_receive):
  data = serial_receive.readline()
  dis = data.decode('utf-8').strip()
  # print(str(uwbNum) +'=' + str(dis))
  while(dis==''):
      data = serial_receive.readline()
      dis = data.decode('utf-8').strip()
      print('Wating:'+str(uwbNum))
      pass
  return float(dis)*100 if data else None

def main(args=None):
    rclpy.init(args=args)
    uwb_init = UWBReceiver()
    rclpy.spin(uwb_init)
    uwb_init.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()