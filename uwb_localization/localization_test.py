import math
import serial
import numpy as np
import cv2
import matplotlib.pyplot as plt
#  from sympy import symbols,  Eq,  nsolve
#  from pykalman import KalmanFilter

# 定義圓球方程式
def f(x,  center,  r):
    return np.array([ (x[0]-center[0][0])**2 + (x[1]-center[0][1])**2 + (x[2]-center[0][2])**2 - r[0]**2   , 
             (x[0]-center[1][0])**2 + (x[1]-center[1][1])**2 + (x[2]-center[1][2])**2 - r[1]**2   , 
             (x[0]-center[2][0])**2 + (x[1]-center[2][1])**2 + (x[2]-center[2][2])**2 - r[2]**2   ])
# 定義方程式的梯度
def grad_f(x,  center):
    return np.array([[ 2*(x[0]-center[0][0]), 2*(x[1]-center[0][1]), 2*(x[2]-center[0][2]) ], 
                     [ 2*(x[0]-center[1][0]), 2*(x[1]-center[1][1]), 2*(x[2]-center[1][2]) ], 
                     [ 2*(x[0]-center[2][0]), 2*(x[1]-center[2][1]), 2*(x[2]-center[2][2]) ]])
# 梯度下降找近似解
def gradient_descent(X,  center,  r):
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
        print("iter {:d} done dx={:}".format(i,  dx))
        #  return [int(X[0]), int(X[1]), int(X[2])]
    # 2D
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
    Q = 1e-5                # Measurement Noise
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

def main():
    # 定義錨點位置
    # [x, y, z]
    center=[
        [360, 360, 60],     # Anchor 0
        [385, 360, 60],     # Anchor 1
        [373, 405, 0]       # Anchor 2
    ]
    # device_tag=serial.Serial("COM14",  115200)
    device_tag=None
    # draw a whole dark image
    image = np.zeros((720, 720, 1),  np.uint8)
    # draw the Anchor Position
    for pos in center:
        cv2.circle(image, (pos[0], pos[1]), 5, 255, 0)
    # Draw Grid
    for i in range(0, 0, 1):
        cv2.line(image, (50*i, 0), (50*i, 720), 255, 1)
        cv2.line(image, (0, 50*i), (720, 50*i), 255, 1)
    cv2.imshow("a", image)
    # Close Image
    if cv2.waitKey(0) & 0xff ==ord('q'):
        return
    
    # main procedure
    print("strat")
    while True:
        # copy an image from the original image
        current_img=image.copy()
        # put tag_pos into temp list
        tag_position=[]
        X = np.array([1, 1, 1])
        # 原本預計找多個近似值X再求平均，但太耗時
        while len(tag_position)!=1:
            # initial circle radus
            # ensure every radius cound be found 
            device_tag.reset_input_buffer()
            r0=[]
            r1=[]
            r2=[]
            # 保證每個錨點都有至少十筆資料
            while len(r0)<10 or len(r1)<10 or len(r2)<10:
                # 從COM取資料
                result=device_tag.readline()
                result=result.decode()
                # 確保資料完整
                if len(result)!=13:
                    continue
                try:
                    # got anchor index and each radius and storing it
                    # 取錨點index
                    anchor_index=int(result[0])
                    # 取與其的距離
                    r_temp=float(result[3:])
                    # 公尺變公分
                    r_temp=r_temp*100
                    if r_temp<0.0 and r_temp>700:
                        continue
                    if anchor_index==0:
                        #  r_temp=1.124*(r_temp**2)+90.82*r_temp-10.28-50-10
                        #  r_temp=96.21*r_temp-14.57-40
                        r0.append(r_temp)
                    elif anchor_index==1:
                        #  r_temp=(-0.118)*(r_temp**2)+79.84*r_temp-14.66+30
                        #  r_temp=79.15*r_temp-13.99
                        #  r_temp=81.68*r_temp-21.85
                        r1.append(r_temp)
                    elif anchor_index==2:
                        #  r_temp=6.21*(r_temp**2)+73.36*r_temp-8.872-50-10# -10
                        #  r_temp=97.58*r_temp-26.28-30+10
                        r2.append(r_temp)
                except:
                    continue
            #  form there store's numbers ,  convert it into numpy
            if find_variance(r0, r1, r2):
                continue

            # 卡爾曼濾波器找平滑
            r0=custom_kalman1D(r0)
            r1=custom_kalman1D(r1)
            r2=custom_kalman1D(r2)
            # 轉成int 因為opencv畫圖只能用int
            r=np.array([r0, r1, r2]).astype(int)
            print("r="+str(r))
            # find local minimum x, y to fit the equtions
            try:
                # 找近似值X
                X = gradient_descent(X, center, r)
            except:
                continue
            print("x= "+str(X))
            tag_position.append(X)
            
        # when tag_pos list is 10 ,  and calc average tag_pos
        #  print(tag_position)
        # 原本預計找多個近似值X再求平均，但太耗時
        tag_position = np.asarray(tag_position)
        tag_position = np.sum(tag_position, axis=0)/1
        tag_position = tag_position.astype(int).reshape((3, ))
 
        # using kalman filter
        #  tag_position=Kalman1D(tag_position)

        #  畫標籤的位置，省略Z軸
        cv2.circle(current_img, tag_position[0:2], 3, 255, 1)
        # 畫標籤到各錨點的線與圓形，並寫上距離
        for i, center_pos in enumerate(center):
            cv2.circle(current_img, center_pos[0:2], r[i], 255, 1)
            cv2.line(current_img, center_pos[0:2], tag_position[0:2], 255, 1)            
            cv2.putText(current_img,  str("{:.1f}".format(np.sqrt(np.sum((center_pos-tag_position)**2)))) , np.array((center_pos[0:2]+tag_position[0:2])/2).astype(int), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 255)
        cv2.imshow("a", current_img)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

if __name__ == '__main__':
    main()