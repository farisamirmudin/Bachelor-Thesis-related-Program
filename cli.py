import os, time, socket, pickle
from serial import Serial
import numpy as np
from function import *
from kalmanfilter import *
from threading import Thread
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path

import platform
if platform.system() == 'Windows':
    port = "COM3"
elif platform.system() == 'Linux':
    port = "/dev/ttyACM0"
elif platform.system() == 'Darwin':
    port = "/dev/tty.usbserial-0001"

home = str(Path.home())

STOPSCAN = '$PW,0,\r\n' # Stop scanning for devices
STARTSCAN = '$PW,1,\r\n' # Start scan
DELCONF = '$PL,\r\n'  # Delete configuration
STARTRANGE = '$PS,\r\n' # Start range
STOPRANGE = '$PG,\r\n' # Stop Ranging
CONF = '$PK,' # Check configuration

class System:
    def __init__(self, dim, kf, anchors):
        self.conn = Serial(port, baudrate = 512000, bytesize = 8, stopbits = 1)
        self.anchors = anchors
        self.kf = kf
        self.dim = dim
        self.zs, self.xs = [], []
        now = datetime.now().strftime("%H-%M-%S")
        self.test_path = '{0}\\position_estimation\\'.format(home)
        os.makedirs(self.test_path)
        os.chdir(self.test_path)
        self.file = open("measurement.txt", 'w+')
        
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
            tmp = resp[2:7] if self.dim == 3 else resp[2:6]
            if '0' not in tmp and len(resp) == 11:
                self.dev = tmp
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
        with open('mc_param.txt', 'r') as f:
            lines = f.readlines()

        for line in lines:
            if 'm' in line:
                m=float(line[4:])
            if 'c' in line:
                c=float(line[4:])

        err = lambda x:m*x+c
        conn.write(STARTRANGE.encode())
        self.reset_input(0.05)
        try:
            self.t1 = time.perf_counter()
            while 1:
                resp = conn.readline().decode('utf-8').split(',')
                tmp = resp[2:6] if self.dim == 3 else resp[2:5]
                if len(resp)==8 and '0' not in tmp:
                    distance = [err(int(x, 16)) for x in tmp]
                    pose = trilateration(self.anchors, distance)
                    if initial:
                        kf.x = pose
                    self.zs.append(pose)
                    kf.predict_update(pose)
                    self.xs.append(kf.x)
                    print("position = ", [np.round(x,2) for x in kf.x])

                    # Write data to file
                    file.write(f"raw position = {pose}\n")
                    file.write(f"distance = {distance}\n")
                    file.write(f"position = {kf.x}\n")

                    # Sends data to client
                    try:
                        msg = pickle.dumps(kf.x)
                        client.send(msg)
                    except:
                        pass
                    initial = False
                    self.reset_input(0.05)

        except KeyboardInterrupt:
            conn.write(STOPRANGE.encode())
            self.save_results()

    def configure(self):
        numTag = int(input('Number of Tag: '))
        numAnchor = int(input('Number of Anchor: '))
        print('Available Devices: ' + ', '.join(self.dev))
        ids = [int(x) for x in input('MA,SA1..SAx,TAG: ').strip().split('')]
        CONF = "$PK,{0},{1},{2},".format(self.dev[ids[0]], numAnchor, numTag)
        for id in ids[1:]:
            CONF = CONF + "{0},".format(self.dev[id])
        CONF = CONF + '\r\n'
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
            plt.savefig('position.png')
        file.close()

def checking_client():
    global client
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('192.168.1.49', 9000))
    s.listen(10)
    while 1:
        client, address = s.accept()
        print(f"Connection from {address} has been established.")

def main():
    dim = int(input('Dimension # 3 for 3D and 2 for 2D: '))
    anchors = []
    for i in range(dim+1):
        tmp = [float(x) for x in input('Coordinate {0} # values separated by comma -> x,y,z: '.format(i+1)).strip().split(',')]
        anchors.append(tmp)

    kf = KalmanFilter(dim=dim)
    kf.R *= 100.
    kf.Q = np.diag([1.5, 1.5, 0.05]) if dim == 3 else np.diag([1.5, 1.5])
    kf.P *= 50.    

    t1 = Thread(target=checking_client)
    t1.daemon = True
    t1.start()

    uwb = System(dim, kf, anchors)
    uwb.reset()
    uwb.scan()
    uwb.configure()
    uwb.ranging()

if __name__ == "__main__":
    main()
