import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from scipy.optimize import fsolve

from geometry_msgs.msg import Vector3


class UwbLocalization(Node):
    # Constants
    kDataCount = 1
    # Dis
    r0 = []
    r1 = []
    r2 = []
    # Anchor Coordinate
    # [x,y,z]
    anchor0 = np.array([0, 0, 60])
    anchor1 = np.array([16, 56, 60])
    anchor2 = np.array([-16, 56, 60])
    # center=[
    #     [360, 360, 60],
    #     [385, 360, 60],     # Anchor 0
    #     [373, 405, 60]     # Anchor 1
    #            # Anchor 2
    # ]
    initial_guess = np.array([50, 50, 60])

    def __init__(self):
        super().__init__('uwb_localization')
        # Subscriber
        self.dis_sub_ = self.create_subscription(Vector3, 'uwb_distance', self.dis_callback, 10)
        self.dis_sub_
        # Publisher
        self.pose_pub_ = self.create_publisher(Vector3, 'uwb_distance', 10)

    # main Loop    
    def dis_callback(self, msgs):
        # self.get_logger().info("Start_localization")
        try:
            # Make sure there are ten dis_data.
            if len(self.r0)<self.kDataCount or len(self.r1)<self.kDataCount or len(self.r2)<self.kDataCount:
                # meter to center meter
                self.r0.append(msgs.x*100)
                self.r1.append(msgs.y*100)
                self.r2.append(msgs.z*100)
                # print(len(self.r0))
            else:
                # Average the distance
                d0 = np.mean(self.r0)
                d1 = np.mean(self.r1)
                d2 = np.mean(self.r2)
                # draw the Anchor Position
                map_image = np.zeros((720, 720, 3), np.uint8)
                # map_image[:] = (128, 128, 128)
                cv2.putText(map_image, '0= '+str(d0), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(map_image, '1= '+str(d1), (100, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(map_image, '2= '+str(d2), (100, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.circle(map_image, (360+self.anchor0[0], 360+self.anchor0[1]), 2, (255,255,255), 0)
                cv2.circle(map_image, (360-self.anchor1[0], 360+self.anchor1[1]), 2, (255,255,255), 0)
                cv2.circle(map_image, (360-self.anchor2[0], 360+self.anchor2[1]), 2, (255,255,255), 0)
                # Define Equation
                def equations(p):
                    x, y, z = p
                    eq1 = (x - self.anchor0[0])**2 + (y - self.anchor0[1])**2 + (z - self.anchor0[2])**2 - d0**2
                    eq2 = (x - self.anchor1[0])**2 + (y - self.anchor1[1])**2 + (z - self.anchor1[2])**2 - d1**2
                    eq3 = (x - self.anchor2[0])**2 + (y - self.anchor2[1])**2 + (z - self.anchor2[2])**2 - d2**2
                    return [eq1, eq2, eq3]
                # # 初始猜測值
                
                # 使用 fsolve 求解
                result = fsolve(equations, self.initial_guess, col_deriv=1)
                # Result
                tag_pos = np.array([int(round(float(result[0]), 2)), int(round(float(result[1]), 2)), int(round(float(result[2]), 2))])
                # print("Target Coordinate:", tag_pos)
                print(result)
                # tag_z = int(round(float(result[2]), 2))
                cv2.circle(map_image, (360+tag_pos[0], 360+tag_pos[1]), 5, (100, 0 , 100), -1)
                # Clear Buffer
                self.r0 = []
                self.r1 = []
                self.r2 = []
                cv2.imshow("Map", map_image)
                cv2.waitKey(1)
                # Update Initial Guess
                self.initial_guess = tag_pos
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    uwb_localization = UwbLocalization()
    rclpy.spin(uwb_localization)
    uwb_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()