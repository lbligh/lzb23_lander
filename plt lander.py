import numpy as np
import matplotlib.pyplot as plt

results = np.loadtxt('trajectory5.txt')
Kh = 18.5e-3

fig, ax1 = plt.subplots()

h = results[:, 1]

theoretical_speed = 0.5+Kh*h

color = 'tab:red'
ax1.set_xlabel('time (s)')
ax1.set_ylabel('height (m)', color=color)
ax1.plot(results[:, 0], results[:, 1], label='h (m)', color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
# we already handled the x-label with ax1
ax2.set_ylabel('descent rate (m/s)', color=color)
ax2.plot(results[:, 0], -1*results[:, 2], label='v (m/s)', color=color)
ax2.plot(results[:, 0], theoretical_speed,
         label='v theoretical (m/s)', color='tab:green')
ax2.tick_params(axis='y', labelcolor=color)
ax2.set_ylim(0, -1.1*np.amin(results[:, 2]))

fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.legend()
plt.show()
