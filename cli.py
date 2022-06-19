import os, time, socket, pickle
from serial import Serial
import numpy as np
from function import *
from kalmanfilter import *
from threading import Thread
import matplotlib.pyplot as plt
from datetime import datetime

import platform
if platform.system() == 'Windows':
    port = "COM3"
elif platform.system() == 'Linux':
    port = "/dev/ttyACM0"
elif platform.system() == 'Darwin':
    port = "/dev/tty.usbserial-0001"

STOPSCAN = '$PW,0,\r\n' # Stop scanning for devices
STARTSCAN = '$PW,1,\r\n' # Start scan
DELCONF = '$PL,\r\n'  # Delete configuration
STARTRANGE = '$PS,\r\n' # Start range
STOPRANGE = '$PG,\r\n' # Stop Ranging
CONF = '$PK,' # Check configuration

class System:
    def __init__(self, kf, anchors):
        self.conn = Serial(port, baudrate = 512000, bytesize = 8, stopbits = 1)
        self.anchors = anchors
        self.kf = kf
        self.zs, self.xs = [], []
        now = datetime.now().strftime("%H-%M-%S")
        self.test_path = f'/Users/faris/Documents/bachelorarbeit/test/3D-positioning/8Jun/{now}'
        os.makedirs(self.test_path)
        self.file = open(f"{self.test_path}/measurement.txt", 'w')
        os.chdir(self.test_path)

    def reset(self):
        conn = self.conn
        conn.write(DELCONF.encode())
        conn.write(STOPSCAN.encode())
        conn.write(STOPRANGE.encode())

    def scan(self):
        conn = self.conn
        conn.write(STARTSCAN.encode()) # Start scanning
        scan = True
        self.reset_input(0.05)
        while scan:
            resp = conn.readline().decode('utf-8').split(',')
            if '0' not in resp[2:7] and len(resp) == 11:
                self.dev = resp[2:7]
                scan = False
        conn.write(STOPSCAN.encode())

    def reset_input(self, t):
        time.sleep(t)
        self.conn.reset_input_buffer()

    def ranging(self):
        kf = self.kf
        file = self.file
        conn = self.conn
        initial = True
        m, b = error_check()
        # err = lambda x:x-(m*x+b)
        err = lambda x:x-25
        conn.write(STARTRANGE.encode())
        self.reset_input(0.05)
        try:
            self.t1 = time.perf_counter()
            while 1:
                resp = conn.readline().decode('utf-8').split(',')
                if len(resp)==8 and '0' not in resp[2:6]:
                    distance = [err(int(x, 16)) for x in resp[2:6]]
                    pose = trilaterate2D(self.anchors, distance)
                    if initial:
                        kf.x[0] = pose[0]
                        kf.x[1] = pose[1]
                    self.zs.append(pose)
                    kf.predict_update(pose)
                    self.xs.append(kf.x)
                    print("position = ", [np.round(x,2) for x in kf.x])

                    # Write data to file
                    file.write(f"distance = {distance}\n")
                    file.write(f"position = {kf.x}\n")

                    # Sends data to client etc. Raspberry pi
                    try:
                        # msg = pickle.dumps(self.anchors)
                        # clientsocket.send(msg)
                        msg = pickle.dumps(kf.x)
                        clientsocket.send(msg)
                    except:
                        pass
                    initial = False
                    self.reset_input(0.05)

        except KeyboardInterrupt:
            conn.write(STOPRANGE.encode())
            self.save_results()

    def configure(self):
        # CONF = "$PK,%s,%d,1," % (dev[idx[0]], len(dev)-2)
        # for i in idx[1:]:
        #     CONF = CONF + "%s," % dev[i]
        # CONF = CONF + '\r\n'
        CONF = "$PK,E1D0,3,1,E1C6,E1A7,E1D4,E1CC,\r\n"
        # CONF = "$PK,E1D0,2,1,E1C6,E1A7,E1CC,\r\n"
        self.conn.write(CONF.encode())

    def save_results(self):
        file = self.file
        self.t2 = time.perf_counter()
        zs, xs = np.array(self.zs), np.array(self.xs)
        std_kf = np.std(xs, axis=0)
        std_raw = np.std(zs, axis=0)
        file.write(f"Total readings = {len(zs)}\n")
        file.write(f"Time={self.t2-self.t1}\n")
        file.write(f"Std kalman={std_kf}\n")
        file.write(f"Std raw={std_raw}\n")
        idx = ['x','y','z']
        for i in range(3):
            plt.figure(i)
            plt.plot(zs[:,i], 'b.')
            plt.plot(xs[:,i], 'r--')
            plt.title(idx[i])
            plt.savefig(f'{self.test_path}/{idx[i]}')
        file.close()

def checking_client():
    global clientsocket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('192.168.1.49', 9000))
    s.listen(10)
    while 1:
        clientsocket, address = s.accept()
        print(f"Connection from {address} has been established.")

def main():
    # anchors = [[0,0,0],[80,0,0],[70,160,0],[-5,140,40]]
    # anchors = [[0,0,0],[114,0,0],[114,80,0],[0,80,0]]
    # anchors = [[0,178,0],[300,80,0],[0,500,0],[300,500,110.5]]
    anchors = [[0,178,0],[300,110,0],[0,500,0],[300,465,82]]
    # anchors = [[0,178,72],[300,110,72],[0,500,72],[300,465,82]] 
    # anchors = [[0,178,72],[300,110,72],[0,500,72]] # 2d

    kf = KalmanFilter(dim=3)
    kf.R *= 100.
    kf.Q = np.diag([1.5, 1.5, 0.05])
    kf.P *= 50.
    # kf = KalmanFilter(dim=2)
    # kf.R *= 100.
    # kf.Q = np.diag([1.5, 1.5])
    # kf.P *= 50.
    

    t1 = Thread(target=checking_client)
    t1.daemon = True
    t1.start()

    uwb = System(kf, anchors)
    uwb.reset()
    uwb.scan()
    uwb.configure()
    uwb.ranging()

if __name__ == "__main__":
    main()
