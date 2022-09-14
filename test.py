import numpy as np

tab = np.load('calibration.npy')
for k in range(4):
    print(tab[k])
