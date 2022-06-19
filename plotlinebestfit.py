from dis import dis
import matplotlib.pyplot as plt
import numpy as np

def plotlinebestfit(distance, error):
    distance = np.array([float(x) for x in distance[1:]])
    error = np.array([float(x) for x in error[1:]])
    cooff = np.polyfit(distance, error, 1)
    p = np.poly1d(cooff)
    plt.plot(distance, error, '.', distance, p(distance))
    plt.title('Error against Distance')
    plt.xlabel("Distance [cm]")
    plt.ylabel('Error [cm]')
    plt.savefig(f"Error against Distance", bbox_inches='tight')