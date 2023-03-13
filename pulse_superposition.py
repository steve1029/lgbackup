import numpy as np
import matplotlib.pyplot as plt

mu = 0
sigma = 10
x = np.arange(-100,100)
y0 = 1/np.sqrt(2*np.pi*sigma) * np.exp(-0.5 * ((x-0)/sigma)**2)
y1 = 1/np.sqrt(2*np.pi*sigma) * np.exp(-0.5 * ((x-20)/sigma)**2)
y2 = 1/np.sqrt(2*np.pi*sigma) * np.exp(-0.5 * ((x-40)/sigma)**2)
y3 = 1/np.sqrt(2*np.pi*sigma) * np.exp(-0.5 * ((x-60)/sigma)**2)
y4 = 1/np.sqrt(2*np.pi*sigma) * np.exp(-0.5 * ((x-80)/sigma)**2)

fig, ax = plt.subplots(nrows=1, ncols=1)

ax.plot(x,y0)
ax.plot(x,y1)
ax.plot(x,y2)
ax.plot(x,y3)
ax.plot(x,y4)
ax.plot(x,y0+y1+y2+y3+y4)
fig.savefig('./pulse_superposition.png')