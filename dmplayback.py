import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
test = '150'
with open(f'C:/Users/Faris/Desktop/stuff/literature/images/test/{test}-cm/measurement.txt') as f:
    lines = f.readlines()

datastring = "".join(lines)
mu = float(lines[-2][3:-1])
start = datastring.index('[')
end = datastring.index(']')
data = datastring[start+1:end].split(' ')
newdata = []
for i in range(0,len(data)):
    if data[i] != '':
        newdata.append(data[i].strip())
data = np.array([int(x) for x in newdata])
sns.set_style('darkgrid')
sns.displot(data)
plt.title(f"Histogram 50cm test")
plt.xlabel("Distance [cm]")
plt.axvline(x=mu, color='r', linestyle='--')
plt.savefig(f"histogram-{test}", bbox_inches='tight')