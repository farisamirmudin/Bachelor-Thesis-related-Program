import numpy as np
from numpy import zeros, eye
class KalmanFilter:
    def __init__(self, dim, N=None) -> None:
        self.N = N
        self.x = zeros(dim)   # state
        self.P = eye(dim)          # uncertainty covariance
        self.Q = eye(dim)          # process uncertainty
        self.R = eye(dim)          # state uncertainty

        self.A = eye(dim)          # state transition matrix
        self.B = eye(dim)

        self.H = eye(dim)   # Measurement function
        self.K = zeros((dim, dim))   # kalman gain
        
        self.I = eye(dim)
        

    def predict_update(self, z, u=None):

        H = self.H
        R = self.R
        A = self.A
        P = self.P
        x = self.x
        Q = self.Q
        B = self.B

        # predict step of normal Kalman filter
        x_pre = A @ x
        if u is not None:
            x_pre += B @ u

        P = A @ P @ A.T + Q

        # update step of normal Kalman filter
        y = z - H @ x_pre

        S = H @ P @ H.T + R
        SI = np.linalg.inv(S)

        K = P @ H @ SI

        x = x_pre + K @ y

        I_KH = self.I - K @ H
        P = I_KH @ P
        self.x = x
        self.P = P