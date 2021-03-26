import numpy as np
import matplotlib.pyplot as plt

# generate some data
ax = np.arange(-9, 10)
X, Y = np.meshgrid(ax, ax)
Z = X ** 2.0 + Y ** 2.0

# normalize the data and convert to uint8 (grayscale conventions)
zNorm = (Z - Z.min()) / (Z.max() - Z.min())
zNormUint8 = zNorm.astype(np.uint8)

# plot result
plt.figure()
plt.imshow(zNormUint8)
plt.show()
