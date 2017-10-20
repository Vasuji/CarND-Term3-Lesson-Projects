import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('train_states.txt', delimiter=',', names=['s', 'd', 's_dot', 'd_dot'])
plt.plot(data['s'], data['d'], 'ro')
plt.show()

# this way we find that the lanes are at d=0, d=4 and d=8
