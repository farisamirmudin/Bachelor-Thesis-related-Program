import serial, time, os, platform
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from datetime import datetime
from scipy.stats import norm

def scan():
    scan = True
    ser.write(b"$PW,1,\r\n")
    while scan:
        ans = ser.readline().decode('utf-8')
        if len(ans) == 30:
            devices = [name for name in ans[6:15].split(',')]
            scan = False
    return devices
    
def configure(devices):
    ma = int(input("Master Anchor: ")) - 1
    tag = int(input("Tag: ")) - 1

    command = "$PK,\r\n"
    print(command)
    ser.write(command.encode())
    print("Configuration Done")
    
def reset():
    ser.write(b'$PW,0,\r\n')
    ser.write(b'$PL,\r\n')
    ser.write(b'$PG,\r\n')

if __name__ == "__main__":
    name_os = platform.system()
    if  name_os == 'Windows':
        port = "COM3"
    elif name_os == 'Darwin':
        port = "/dev/cu.usbserial-0001"
    elif name_os == 'Linux':
        port = "/dev/ttyACM0"
    ser = serial.Serial(port = port, baudrate = 512000, bytesize = 8, stopbits = 1)
    dist = '500'
    now = datetime.now().strftime("%H-%M-%S")
    test_path = f'/Users/faris/Documents/test/two-devices/{dist}-cm/{now}'
    os.makedirs(test_path)
    f = open(f"{test_path}/measurement.txt", 'w')
    os.chdir(test_path)
    CONF = "$PK,E1B3,0,1,E1C6,\r\n"
    ser.write(CONF.encode())
    time.sleep(0.05)
    ser.write(b"$PS,\r\n")
    time.sleep(0.05)
    ser.reset_input_buffer()
    xs = []
    while len(xs)!=100:
        resp = ser.readline().decode().split(',')
        if len(resp) == 8 and resp[2] != '0':
            val = int(resp[2], 16)
            print(val)
            xs.append(val)
    ser.write(b"$PG,\r\n")
    xs = np.array(xs)
    mu = np.mean(xs)
    sigma = np.std(xs)
    x = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
    plt.plot(x, norm.pdf(x,mu,sigma))
    plt.axvline(x=mu, color='r', linestyle='--')
    plt.title(f"{dist} cm Test")
    plt.xlabel("Distance [cm]")
    plt.savefig(f"gaussian-{dist}", bbox_inches='tight')
    sns.set_style('darkgrid')
    sns.displot(xs)
    plt.title(f"Histogram {dist}cm test")
    plt.xlabel("Distance [cm]")
    plt.axvline(x=mu, color='r', linestyle='--')
    plt.savefig(f"histogram-{dist}", bbox_inches='tight')
    plt.show()
    f.write(f"data={xs}\n")
    f.write(f"mu={mu}\n")
    f.write(f"Std={sigma}\n")
    f.close()



