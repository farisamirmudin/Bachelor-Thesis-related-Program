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
        self.now = datetime.now().strftime(f"%d-%m-%y %H-%M-%S")
        os.mkdir(self.now)
        self.file = open(f"{self.now}/measurement.txt", 'w+')
        
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
            # print('scan')
            resp = conn.readline().decode('utf-8').split(',')
            # print(resp)
            tmp = resp[2:7] if self.dim == 3 else resp[2:6]
            if '0' not in tmp and len(resp) == 11:
                self.dev = tmp
                scan = False
        conn.write(STOPSCAN.encode())

    def configure(self, existing_conf = None):
        if existing_conf:
            existing_conf += '\r\n'
            self.conn.write(existing_conf.encode())
            return
        numTag = int(input('Number of Tag: '))
        numAnchor = int(input('Number of Anchor: '))
        dev_str = 'Available Devices: '
        for idx, dev in enumerate(self.dev):
            dev_str += '[{0}]{1} '.format(idx, dev)
        print(dev_str)
        ids = [int(x) for x in input('Write the index to assign devices [ma,sa1..sax,tag]: ').strip().split(',')]
        CONF = "$PK,{0},{1},{2},".format(self.dev[ids[0]], numAnchor-1, numTag)
        for id in ids[1:]:
            CONF = CONF + "{0},".format(self.dev[id])
        with open('anchor_coordinate.txt', 'a') as f:
            f.write(CONF)
        CONF = CONF + '\r\n'
        self.conn.write(CONF.encode())

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
            m=float(lines[0][4:])
            c=float(lines[1][4:])
            # print("m={0},c={1}".format(m,c))

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
                    pose = trilateration(self.anchors, distance, self.dim)
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
                    if clients:
                        msg = pickle.dumps(kf.x)
                        for client in clients:
                            client.send(msg)
                    initial = False
                    self.reset_input(0.05)

        except KeyboardInterrupt:
            conn.write(STOPRANGE.encode())
            self.save_results()

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
        for i in range(self.dim):
            plt.figure(i)
            plt.plot(zs[:,i], 'b.')
            plt.plot(xs[:,i], 'r--')
            plt.title(idx[i])
            plt.savefig('{0}/{1}.png'.format(self.now, idx[i]))
        file.close()

def checking_client(anchors):
    global clients
    msg = pickle.dumps(anchors)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('192.168.1.50', 9000))
    s.listen(10)
    clients = {}
    while 1:
        client, address = s.accept()
        clients[client] = 0
        print(f"Connection from {address} has been established.")
        for client in clients:
            if clients[client] == 0:
                client.send(msg)
                clients[client] = 1

def ask_anch_coordinate(dim):
    file_tmp = open('anchor_coordinate.txt', 'w')
    anchors = []
    str_to_ask = 'Coordinate {0} # values separated by comma -> x,y: ' if dim == 2 else 'Coordinate {0} # values separated by comma -> x,y,z: '
    for i in range(dim+1):
        if dim == 2:
            resp = input(str_to_ask.format(i+1)).strip()
            anchor = [float(x) for x in resp.split(',')]
        else:
            resp = input(str_to_ask.format(i+1)).strip()
            anchor = [float(x) for x in resp.split(',')]
        file_tmp.write(resp + '\n')
        anchors.append(anchor)
    return anchors

if __name__ == "__main__":
    os.chdir(os.path.dirname(__file__)) 
    dim = int(input('Dimension [2,3]? '))
    anchors = []
    existing_cmd = ''
    if os.path.exists('anchor_coordinate.txt'):
        resp_usr = input("Configuration already existed. Proceed with existing configuration or reset [p,r]? ")
        if resp_usr == 'r':
            os.remove('anchor_coordinate.txt')
        else:
            with open('anchor_coordinate.txt', 'r') as f:
                lines = f.readlines()
            existing_cmd = lines[-1]
            for line in lines[:-1]:
                anchor = [float(x) for x in line.split(',')]
                anchors.append(anchor)
    if not os.path.exists('anchor_coordinate.txt'):
        anchors = ask_anch_coordinate(dim)
    
    kf = KalmanFilter(dim=dim)
    kf.R *= 100.
    kf.Q = np.diag([1., 1., 1.]) if dim == 3 else np.diag([1.5, 1.5])
    kf.P *= 50.    

    t1 = Thread(target=checking_client)
    t1.daemon = True
    t1.start()

    uwb = System(dim, kf, anchors)
    print("Resetting...")
    uwb.reset()
    print("Scanning...")
    uwb.scan()
    print("Configuring...")
    uwb.configure(existing_cmd)
    print("Ranging...")
    uwb.ranging()
