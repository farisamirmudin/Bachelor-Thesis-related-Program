import os, time, socket, pickle
from serial import Serial
import numpy as np
from function import *
from kalmanfilter import *
from threading import Thread
import matplotlib.pyplot as plt
from datetime import datetime

                    

file = '/in circle/3z 0 no obst/'
saveto = 'C:/Users/faris/Documents/pmtest{0}'.format(file)
if not os.path.isdir(saveto):
    os.makedirs(saveto)
pathtmp = 'C:/Users/faris/Documents/bachelorarbeit/test/3D-positioning/8Jun{}measurement.txt'.format(file)
def save_results():
    t2 = time.perf_counter()
    zs, xs = np.array(zs), np.array(xs)
    std_kf = np.std(xs, axis=0)
    std_raw = np.std(zs, axis=0)
    file.write(f"Total readings = {len(zs)}\n")
    file.write(f"Time={t2-t1}\n")
    file.write(f"Std kalman={std_kf}\n")
    file.write(f"Std raw={std_raw}\n")
    idx = ['x','y','z']
    for i in range(3):
        plt.figure(i)
        plt.plot(zs[:,i], 'b.')
        plt.plot(xs[:,i], 'r--')
        plt.title(idx[i])
        plt.savefig(f'{saveto}/{idx[i]}')
    file.close()


with open(pathtmp, 'r') as f:
    data = f.readlines()
postmp = []
for line in data:
    if 'position' in line:
        tmp = line[12:-2].strip().split(' ')
        tmp2 = [float(x) for x in tmp if x != '']
        postmp.append(tmp2)
pos = postmp[:1000]
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('192.168.178.20', 9000))
s.listen(10)
done = False
while not done:
    clientsocket, address = s.accept()
    print(f"Connection from {address} has been established.")
    t1 = time.perf_counter()
    for p in pos:
        msg = pickle.dumps(p)
        clientsocket.send(msg)
        time.sleep(0.05)
    done = True

