import rclpy
from rclpy.node import Node
import serial
import numpy as np

from geometry_msgs.msg import Vector3
from std_msgs.msg import String

class custom_kalman1D:
  def __init__(self):
    n = 10
    self.Q = 1e-5  # 過程噪聲協方差
    self.R = 0.01**2  # 觀測噪聲協方差
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
    self.P[0] = 1.0

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

  a0_kalman = custom_kalman1D()
  a1_kalman = custom_kalman1D()
  a2_kalman = custom_kalman1D()

  baudRate = 115200
  d0 = []
  d1 = []
  d2 = []
  listNum = 3
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
        dis_anchor0 = send_and_receive(i, self.serial_uwb0)*100
        if dis_anchor0<= 0:
            dis_msgs.x = self.last_0
        else:
            dis_anchor0 = self.a0_kalman.renew_and_getdata(dis_anchor0)
            dis_msgs.x = dis_anchor0
            self.last_0 = dis_anchor0
        i=1
        dis_anchor1 = send_and_receive(i, self.serial_uwb1)*100
        if dis_anchor1<= 0:
            dis_msgs.y = self.last_1
        else:
            dis_anchor1 = self.a1_kalman.renew_and_getdata(dis_anchor1)
            dis_msgs.y = dis_anchor1
            self.last_1 = dis_anchor1
        i=2
        dis_anchor2 = send_and_receive(i, self.serial_uwb2)*100
        if dis_anchor2 <= 0:
            dis_msgs.z = self.last_2
        else:
            dis_anchor2 = self.a2_kalman.renew_and_getdata(dis_anchor2)
            dis_msgs.z = dis_anchor2
            self.last_2 = dis_anchor2
        self.pose_pub_.publish(dis_msgs)
    except KeyboardInterrupt:
      self.serial_uwb0.close()
      self.serial_uwb1.close()
      self.serial_uwb2.close()
      print("Program terminated by user.")

def send_and_receive(uwbNum, serial_receive):
  try:
    data = serial_receive.readline()
    dis = data.decode('utf-8').strip()
    while(dis==''):
        data = serial_receive.readline()
        dis = data.decode('utf-8').strip()
        # print('dis=' + str(dis))
        # print('No Data, Wating:'+str(uwbNum))
    return float(dis)
  except:
    pass

def main(args=None):
    rclpy.init(args=args)
    uwb_init = UWBReceiver()
    rclpy.spin(uwb_init)
    uwb_init.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()