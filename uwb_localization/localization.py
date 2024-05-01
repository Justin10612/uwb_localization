import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from geometry_msgs.msg import Vector3


class UwbLocalization(Node):
    # Constants
    kDataCount = 1
    # Dis
    r0 = []
    r1 = []
    r2 = []
    tx = []
    ty = []
    # Anchor Coordinate
    # [x,y,z]
    anchor = np.array([[0, 0, 22],
                      [16, 56, 0],
                      [-16, 56, 0]])
    anchor0 = np.array([0, 0, 22])
    anchor1 = np.array([16, 56, 2])
    anchor2 = np.array([-16, 56, -2])
    # initail guess
    initial_guess = np.array([1, 1, 1])

    def __init__(self):
      super().__init__('d_localization')
      # Subscriber
      self.dis_sub_ = self.create_subscription(Vector3, 'uwb_distance', self.dis_callback, 10)
      self.dis_sub_
      # Publisher
      self.pose_pub_ = self.create_publisher(Vector3, 'human_pose', 10)

    # main Loop    
    def dis_callback(self, msgs):
      # self.get_logger().info("Start_localization")
      try:
        # Make sure there are ten dis_data.
        if len(self.r0)<self.kDataCount or len(self.r1)<self.kDataCount or len(self.r2)<self.kDataCount:
          # meter to center meter
          self.r0.append(msgs.x)
          self.r1.append(msgs.y)
          self.r2.append(msgs.z)
          # print(len(self.r0))
        else:
          # Average the distance
          # d0 = np.mean(self.r0)
          # d1 = np.mean(self.r1)
          # d2 = np.mean(self.r2)
          d0 = msgs.x
          d1 = msgs.y
          d2 = msgs.z
          r = np.array([d0, d1, d2]).astype(int)
          # draw the Anchor Position
          map_image = np.zeros((720, 720, 3), np.uint8)
          # map_image[:] = (128, 128, 128)
          cv2.putText(map_image, 'A0 = '+str(round(d0, 2)), (360+self.anchor0[0], 300+self.anchor0[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
          cv2.putText(map_image, 'A1 = '+str(round(d1, 2)), (280-self.anchor1[0], 420+self.anchor1[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
          cv2.putText(map_image, 'A2 = '+str(round(d2, 2)), (360-self.anchor2[0], 420+self.anchor2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
          cv2.circle(map_image, (360+self.anchor0[0], 360+self.anchor0[1]), 2, (255,255,255), 0)
          cv2.circle(map_image, (360-self.anchor1[0], 360+self.anchor1[1]), 2, (255,255,255), 0)
          cv2.circle(map_image, (360-self.anchor2[0], 360+self.anchor2[1]), 2, (255,255,255), 0)
          cv2.circle(map_image, (360+self.anchor0[0], 360+self.anchor0[1]), int(d0), (255,0,10), 0)
          cv2.circle(map_image, (360-self.anchor1[0], 360+self.anchor1[1]), int(d1), (0,255,10), 0)
          cv2.circle(map_image, (360-self.anchor2[0], 360+self.anchor2[1]), int(d2), (0,10,255), 0)
          # Find coordinate
          try:
            self.initial_guess=gradient_descent(self.initial_guess, self.anchor, r)
          except:
            pass
          # Result
          tag_pos = np.array([int(round(float(self.initial_guess[0]), 2)), 
                              int(round(float(self.initial_guess[1]), 2))])
          print(tag_pos)
          # tag_z = int(round(float(result[2]), 2))
          cv2.circle(map_image, (360-tag_pos[0], 360+tag_pos[1]), 5, (100, 50 , 100), -1)
          # Clear Buffer
          self.r0 = []
          self.r1 = []
          self.r2 = []
          cv2.imshow("Map", map_image)
          cv2.waitKey(1)
          # Update Initial Guess
      except KeyboardInterrupt:
        cv2.destroyAllWindows()

#定義圓球方程式
def f(x, center, r):
  return np.array([
    (x[0]-center[0][0])**2 + (x[1]-center[0][1])**2 + (x[2]-center[0][2])**2 - r[0]**2,
    (x[0]-center[1][0])**2 + (x[1]-center[1][1])**2 + (x[2]-center[1][2])**2 - r[1]**2,
    (x[0]-center[2][0])**2 + (x[1]-center[2][1])**2 + (x[2]-center[2][2])**2 - r[2]**2])
#定義方程式的梯度
def grad_f(x, center):
  return np.array([
    [2*(x[0]-center[0][0]) , 2*(x[1]-center[0][1]) , 2*(x[2]-center[0][2]) ],
    [2*(x[0]-center[1][0]) , 2*(x[1]-center[1][1]) , 2*(x[2]-center[1][2]) ],
    [2*(x[0]-center[2][0]) , 2*(x[1]-center[2][1]) , 2*(x[2]-center[2][2]) ]])
#梯度下降找近似解
def gradient_descent(X,center,r):
  esp=1e-2
  N=300
  for i in range(N):
    # print("iter : {:d}".format(i))
    fx = f(X,center,r)
    grad = grad_f(X,center)
    dx = np.dot(grad.T, fx)
    X = X-(9e-7)*dx
    if  np.sqrt(np.sum(dx**2)) < esp:
        break
    # print("iter {:d} done dx={:}".format(i, dx))
  #2D
  return [int(X[0]),int(X[1]),int(X[2])]

def main(args=None):
    rclpy.init(args=args)
    uwb_localization = UwbLocalization()
    rclpy.spin(uwb_localization)
    uwb_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()