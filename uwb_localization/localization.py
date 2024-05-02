import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from geometry_msgs.msg import Vector3


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
    self.xhat_minus = np.zeros(n)  # 預測值
    self.xhat_minus =[0]
    self.Pminus = np.zeros(n)  # 預測值的協方差
    self.Pminus =[0]
    self.K = np.zeros(n)  # 卡爾曼增益
    self.K =[0]
    self.xhat[0] = 0.0
    self.P[0] = 100.0

  def renew_and_getdata(self, raw_data):
    # 預測
    # xhat_minus[k] = xhat[k-1]
    self.xhat_minus[0]=self.xhat[0]
    # Pminus[k] = P[k-1] + Q
    self.Pminus[0]=self.P[0]+self.Q
    # 更新
    # K[k] = Pminus[k] / (Pminus[k] + R)
    self.K[0]=self.Pminus[0]/(self.Pminus[0]+self.R)
    # xhat[k] = xhatminus[k] + K[k] * (data[k] - xhatminus[k])
    self.xhat[1]=self.xhat_minus[0]+self.K[0]*(raw_data-self.xhat_minus[0])
    # P[k] = (1 - K[k]) * Pminus[k]
    self.P[0] = (1 - self.K[0]) * self.Pminus[0]
    self.xhat[0]=self.xhat[1]
    return self.xhat[0]


class UwbLocalization(Node):
    # Anchor Coordinate
    # [x,y,z]
    anchor = np.array([[0, 0, 21],
                      [16, 56, 0],
                      [-16, 56, 0]])
    # initail guess
    initial_guess = np.array([1, 100, 10])

    def __init__(self):
      super().__init__('d_localization')
      # Subscriber
      self.dis_sub_ = self.create_subscription(Vector3, 'uwb_distance', self.dis_callback, 10)
      self.dis_sub_
      # Publisher
      self.pose_pub_ = self.create_publisher(Vector3, 'human_pose_distance', 10)

    # main Loop    
    def dis_callback(self, msgs):
      # self.get_logger().info("Start_localization")
      pose_msgs = Vector3()
      try:
        d0 = 0.9766*msgs.x + 35.253
        d1 = 0.9738*msgs.y  + 26.868
        d2 = 0.9387*msgs.z + 30.364
        r = np.array([d0, d1, d2]).astype(int)
        # draw the Anchor Position
        map_image = np.zeros((720, 720, 3), np.uint8)
        # map_image[:] = (128, 128, 128)
        cv2.putText(map_image, 'A0 = '+str(r[0]), (360+self.anchor[0][0], 300+self.anchor[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(map_image, 'A1 = '+str(r[1]), (280-self.anchor[1][0], 420+self.anchor[1][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(map_image, 'A2 = '+str(r[2]), (360-self.anchor[2][0], 420+self.anchor[2][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(map_image, (360+self.anchor[0][0], 360+self.anchor[0][1]), 2, (255,255,255), 0)
        cv2.circle(map_image, (360-self.anchor[1][0], 360+self.anchor[1][1]), 2, (255,255,255), 0)
        cv2.circle(map_image, (360-self.anchor[2][0], 360+self.anchor[2][1]), 2, (255,255,255), 0)
        cv2.circle(map_image, (360+self.anchor[0][0], 360+self.anchor[0][1]), r[0], (205,153,0), 0)
        cv2.circle(map_image, (360-self.anchor[1][0], 360+self.anchor[1][1]), r[1], (0,255,10), 0)
        cv2.circle(map_image, (360-self.anchor[2][0], 360+self.anchor[2][1]), r[2], (0,10,255), 0)
        # Find coordinate
        try:
          self.initial_guess=gradient_descent(self.initial_guess, self.anchor, r)
        except:
          pass
        # Result
        tag_pos = np.array([int(round(self.initial_guess[0], 2)), 
                            int(round(self.initial_guess[1], 2)),
                            int(round(self.initial_guess[2], 2))])
        
        print(tag_pos)
        pose_msgs.x = float(tag_pos[0])
        pose_msgs.y = float(tag_pos[1])
        pose_msgs.z = float(tag_pos[2])
        self.pose_pub_.publish(pose_msgs)
        # Draw
        cv2.putText(map_image, 'Distance = '+str(np.sqrt(tag_pos[0]**2+tag_pos[1]**2)), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.arrowedLine(map_image, (360+self.anchor[0][0], 360+self.anchor[0][1]), (360-tag_pos[0], 360+tag_pos[1]), (8, 255, 255), 2, 2, 0, 0.05)
        cv2.circle(map_image, (360-tag_pos[0], 360+tag_pos[1]), 5, (0, 255 , 100), -1)
        cv2.imshow("Map", map_image)
        cv2.waitKey(1)
      except KeyboardInterrupt:
        cv2.destroyAllWindows()

#定義圓球方程式
def f(P, center, r):
  return np.array([
    (P[0]-center[0][0])**2 + (P[1]-center[0][1])**2 + (P[2]-center[0][2])**2 - r[0]**2,
    (P[0]-center[1][0])**2 + (P[1]-center[1][1])**2 + (P[2]-center[1][2])**2 - r[1]**2,
    (P[0]-center[2][0])**2 + (P[1]-center[2][1])**2 + (P[2]-center[2][2])**2 - r[2]**2])
#定義方程式的梯度
def grad_f(x, center):
  return np.array([
    [2*(x[0]-center[0][0]) , 2*(x[1]-center[0][1]) , 2*(x[2]-center[0][2])],
    [2*(x[0]-center[1][0]) , 2*(x[1]-center[1][1]) , 2*(x[2]-center[1][2])],
    [2*(x[0]-center[2][0]) , 2*(x[1]-center[2][1]) , 2*(x[2]-center[2][2])]])
#梯度下降找近似解
def gradient_descent(X, center, r):
  esp=1e-2
  N=3000
  for i in range(N):
    fx = f(X, center,r) 
    grad = grad_f(X, center)
    dx = np.dot(grad.T, fx)
    # print(dx)
    # # 檢查 dx 是否導致 Z 值為負，如果是，將其設置為一個非常大的值
    # if dx[2] < 0:
    #     X[2] = np.abs(X[2])
    # X = X - (9e-7)*dx
    X[0] = X[0] - (6e-6)*dx[0]
    X[1] = X[1] - (6e-6)*dx[1]
    X[2] = X[2] - (5e-6)*dx[2]
    if  np.sqrt(np.sum(dx**2)) < esp:
        break
    # print("y : {:2f}, z : {:2f}".format(X[1], X[2]))
  # print("iter : {:d}".format(i))
  # return [int(X[0]), int(X[1]), int(X[2])]
  return [X[0], X[1], X[2]]

def main(args=None):
    rclpy.init(args=args)
    uwb_localization = UwbLocalization()
    rclpy.spin(uwb_localization)
    uwb_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()