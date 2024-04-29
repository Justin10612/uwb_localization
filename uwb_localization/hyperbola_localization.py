import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from scipy.optimize import fsolve

from geometry_msgs.msg import Vector3


class HyperbolaLocalization(Node):
    # Constants
    kDataCount = 2
    # Dis
    r0 = []
    r1 = []
    r2 = []
    # Anchor Coordinate
    anchor0 = np.array([0, 0, 60])
    anchor1 = np.array([16, 56, 60])
    anchor2 = np.array([-16, 56, 60])

    def __init__(self):
        super().__init__('hyperbola_localization')
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
                cv2.putText(map_image, 'r0 = '+str(d0), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(map_image, 'r1 = '+str(d1), (100, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(map_image, 'r2 = '+str(d2), (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.circle(map_image, (360+self.anchor0[0], 360+self.anchor0[1]), 2, (255,255,255), 0)
                cv2.circle(map_image, (360-self.anchor1[0], 360+self.anchor1[1]), 2, (255,255,255), 0)
                cv2.circle(map_image, (360-self.anchor2[0], 360+self.anchor2[1]), 2, (255,255,255), 0)
                # Equation
                ### 計算sin cos
                side1 = 32
                cosine_phi = (side1 ** 2 + d1 ** 2 - d2 ** 2) / (2 * d1 * side1)
                if cosine_phi > 1:
                    cosine_phi = 1
                if cosine_phi < -1:
                    cosine_phi = -1
                print("cos = "+str(cosine_phi))
                sin_phi = np.sqrt(1-cosine_phi**2)
                print("sin = "+str(sin_phi))
                ### 計算點一
                xT1 = self.anchor1[0]-d1*cosine_phi
                yT1 = d1*sin_phi
                ### 計算點二
                xT2 = self.anchor1[0]-d1*cosine_phi
                yT2 = -d1*sin_phi
                ### 計算距離
                DT1 = np.sqrt((xT1-self.anchor0[0])**2 + (yT1-self.anchor0[1])**2)
                DT2 = np.sqrt((xT2-self.anchor0[0])**2 + (yT2-self.anchor0[1])**2)
                print("Dt1 = " + str(DT1))
                print("Dt2 = " + str(DT2))
                ### 比較
                if np.abs(DT1-d0)<np.abs(DT2-d0) :
                    tag_pos = np.array([int(xT1), int(yT1)])
                else:
                    tag_pos = np.array([int(xT2), int(yT2)])

                print("Target Coordinate:", tag_pos)

                cv2.circle(map_image, (360+tag_pos[0], 360+tag_pos[1]), 5, (100, 0 , 100), -1)
                # Clear Buffer
                self.r0 = []
                self.r1 = []
                self.r2 = []
                cv2.imshow("Map", map_image)
                cv2.waitKey(1)
                # Update Initial Guess
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    uwb_localization = HyperbolaLocalization()
    rclpy.spin(uwb_localization)
    uwb_localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()