import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import math

from geometry_msgs.msg import Vector3
from std_msgs.msg import String


class UwbLocalization(Node):
    # Constants
    kDataCount = 3
    # Dis
    r0 = []
    r1 = []
    r2 = []
    # Anchor Coordinate
    # [x,y,z]
    center=[
        [360, 360, 60],
        [385, 360, 60],     # Anchor 0
        [373, 405, 60]     # Anchor 1
               # Anchor 2
    ]
    def __init__(self):
        super().__init__('uwb_localization')
        # Subscriber
        self.dis_sub_ = self.create_subscription(Vector3, 'uwb_distance', self.dis_callback, 10)
        self.dis_sub_
        # Publisher
        self.pose_pub_ = self.create_publisher(Vector3, 'uwb_distance', 10)

    
    # main Loop    
    def dis_callback(self, msgs):
        self.get_logger().info("Start_localization")
        # put tag_pos into temp list
        tag_position=[]
        r_array = []
        tag_position = np.array([1, 1, 1])
        try:
            # Make sure there are ten dis_data.
            if len(self.r0)<self.kDataCount or len(self.r1)<self.kDataCount or len(self.r2)<self.kDataCount:
                # meter to center meter
                self.r0.append(msgs.x*100)
                self.r1.append(msgs.y*100)
                self.r2.append(msgs.z*100)
                print(len(self.r0))
            else:
                map_image = np.zeros((720, 720, 1), np.uint8)
                # draw the Anchor Position
                for pos in self.center:
                    cv2.circle(map_image, (pos[0], pos[1]), 2, 255, 0)
                # # kalman Filter
                self.r0 = custom_kalman1D(self.r0)
                self.r1 = custom_kalman1D(self.r1)
                self.r2 = custom_kalman1D(self.r2)
                # 轉成int
                r_array = np.array([self.r0, self.r1, self.r2]).astype(int)
                print("r_array = "+str(r_array))
                # find local minimum x, y to fit the equtions
                # 找近似值X
                tag_position = gradient_descent(tag_position, self.center, r_array)
                print("Tag_position = "+str(tag_position))
                print("tag 2D position" + str(tag_position[0:2]))
                # 2D Distance between Anchors and the tag
                dis0 = round(math.hypot(tag_position[0]-self.center[0][0], tag_position[1]-self.center[0][1]), 2)
                dis1 = round(math.hypot(tag_position[0]-self.center[1][0], tag_position[1]-self.center[1][1]), 2)
                dis2 = round(math.hypot(tag_position[0]-self.center[2][0], tag_position[1]-self.center[2][1]), 2)
                # Plot point
                cv2.circle(map_image, tag_position[0:2], 3, 255, 1)
                cv2.putText(map_image, str(dis0), (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 255)
                cv2.putText(map_image, str(dis1), (200, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 255)
                cv2.putText(map_image, str(dis2), (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 255)
                # # 畫標籤到各錨點的線與圓形，並寫上距離
                # for i, center_pos in enumerate(self.center):
                #     # cv2.circle(map_image, center_pos[0:2], r_array[i], 255, 1)
                #     cv2.line(map_image, center_pos[0:2], tag_position[0:2], 255, 1)            
                #     cv2.putText(map_image, str("{:.1f}".format(np.sqrt(np.sum((center_pos-tag_position)**2)))), np.array((center_pos[0:2]+tag_position[0:2])/2).astype(int), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 255)
                # Clear Buffer
                self.r0 = []
                self.r1 = []
                self.r2 = []
                cv2.imshow("Map", map_image)
                cv2.waitKey(1)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()
        
# 定義圓球方程式
def f(x, center, r):
    return np.array([
        (x[0]-center[0][0])**2 + (x[1]-center[0][1])**2 + (x[2]-center[0][2])**2 - r[0]**2, 
        (x[0]-center[1][0])**2 + (x[1]-center[1][1])**2 + (x[2]-center[1][2])**2 - r[1]**2, 
        (x[0]-center[2][0])**2 + (x[1]-center[2][1])**2 + (x[2]-center[2][2])**2 - r[2]**2])
# 定義方程式的梯度
def grad_f(x,  center):
    return np.array([
        [2*(x[0]-center[0][0]), 2*(x[1]-center[0][1]), 2*(x[2]-center[0][2])], 
        [2*(x[0]-center[1][0]), 2*(x[1]-center[1][1]), 2*(x[2]-center[1][2])], 
        [2*(x[0]-center[2][0]), 2*(x[1]-center[2][1]), 2*(x[2]-center[2][2])]])
# 梯度下降找近似解
def gradient_descent(X, center, r):
    esp = 1e-2
    N = 300
    for i in range(N):
        #  print("iter : {:d}".format(i))
        fx = f(X, center, r)
        grad = grad_f(X, center)
        dx = np.dot(grad.T, fx)
        X = X-(9e-7)*dx
        if  np.sqrt(np.sum(dx**2)) < esp:
            break
    return [int(X[0]), int(X[1]), int(X[2])]

# 找變異數
def find_variance(r0, r1, r2):
    r0 = np.asarray(r0)
    r1 = np.asarray(r1)
    r2 = np.asarray(r2)
    variance0=np.var(r0)
    variance1=np.var(r1)
    variance2=np.var(r2)
    #  print(variance0, variance1, variance2)
    if variance0 > 300 or variance1 >300 or variance2>300:
        return True
    
#  Kalman Filter
def custom_kalman1D(observations):
    #  假設你有一組一維的資料
    #  Initialize
    n = len(observations)   # Process Noise
    Q = 5e-3                # Measurement Noise
    R = 0.01**2
    xhat = np.zeros(n)  # 估計值
    P = np.zeros(n)     # 估計值的協方差
    xhatminus = np.zeros(n) # 預測值
    Pminus = np.zeros(n)    # 預測值的協方差
    K = np.zeros(n)         #  卡爾曼增益
    #  Init
    xhat[0] = observations[0]
    P[0] = 1.0
    for k in range(1, n):
        #  Predict
        xhatminus[k] = xhat[k-1]
        Pminus[k] = P[k-1] + Q
        #  Update
        K[k] = Pminus[k] / (Pminus[k] + R)
        xhat[k] = xhatminus[k] + K[k] * (observations[k] - xhatminus[k])
        P[k] = (1 - K[k]) * Pminus[k]
    #  xhat=np.sum(xhat)/len(xhat)
    return xhat[-1]

def main(args=None):
    rclpy.init(args=args)
    uwb_localization = UwbLocalization()
    rclpy.spin(uwb_localization)
    uwb_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()