import numpy as np
from scipy.signal import square
from kalman_filter import KalmanFilter as KF


class IP:
    def __init__(self,alpha = 0.25, kp = 100.0, ki = 0.05, dt = 0.001):

        # iPD control parameters
        self.alpha = alpha  # Control gain
        self.Kp = kp  # Proportional gainn
        self.Ki = ki  # Integral gain
        self.dt = dt
        self.integral_error = 0

        #Ki = .01 # Integral Gain
        # Noise 
        # sigmax = 0.1
        # sigmav = 0.01 


    # Kalman filter for position 

    # F = np.array([
    #     [1, dt, 1/2*dt**2],
    #     [0, 1, dt],
    #     [0, 0, 1]
    # ])

    # # Observing just the position
    # H = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]]).reshape(3, 3)
    # H = np.array([[1, 0, 0],[0, 0, 1]]).reshape(2, 3)

    # # Noise covariance 
    # Q = np.diag([.01,sigmav,.0001])

    # R = np.eye(3) * sigmax
    # R = np.eye(2) * sigmax

    # #initialise the filter
    # kf = KF(F = F, H = H, Q = Q, R = R)


    #estimating F 

        Fkalm = np.array([
        [1, dt, 1/2*dt**2],
        [0, 1, dt],
        [0, 0, 1]
        ])

        Bkalm = np.array([
        [self.alpha*1/2*dt**2, self.alpha*dt,0]
        ]).reshape(3,1)

        # Observing position and velocity
        Hkalm = np.array([[1, 0, 0],[0, 1, 0]]).reshape(2, 3)

        # Noise covariance 
        Qkalm = np.diag([10,1,.1])/100

        Rkalm = np.diag([.1,10]) * 1

        #initialise the filter

        self.kalmF = KF(F = Fkalm, H=Hkalm, Q=Qkalm, R=Rkalm ,B=Bkalm)
        self.u = 0
        self.ref_prev = 0

    def control(self,x,x_ref):
        # iPD Control loop
        x_ref_dot = (x_ref - self.ref_prev)/self.dt
        error = x - x_ref 
        self.integral_error += error * self.dt
        self.kalmF.predict(self.u)
        self.kalmF.update(np.array([x_ref,float(x_ref_dot)]).reshape(2,1))
        F_estimated = self.kalmF.x[2,0]
        self.u =  (F_estimated - x_ref_dot + self.Kp*error + self.Ki * self.integral_error)*(1/self.alpha)
        self.ref_prev = x_ref
        return self.u

