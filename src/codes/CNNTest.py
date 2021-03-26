import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import tensorflow as tf
from tensorflow import keras
from sklearn.model_selection import train_test_split
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
from numpy import savetxt
from numpy import save
from numpy import load

data_x = load('/home/laptop/codes/data_x2.npy')
data_y = load('/home/laptop/codes/data_y2.npy')

def plot_image(i, predictions_array, true_labels, images):
    predictions_array, true_label, img = predictions_array[i], true_labels[i], images
    plt.grid(False)
    plt.xticks([])
    plt.yticks([])

    plt.imshow(img)

    predicted_label = np.argmax(predictions_array)
    if predicted_label == true_label:
        color = 'blue'
    else:
        color = 'red'

    plt.xlabel("{} {:2.0f}% ({})".format(predicted_label,
                                100*np.max(predictions_array),
                                true_label),
                                color=color)

def plot_value_array(i, predictions_array, true_label):
    predictions_array, true_label = predictions_array[i], true_label[i]
    plt.grid(False)
    plt.xticks([])
    plt.yticks([])
    thisplot = plt.bar(range(5), predictions_array, color="#777777")
    plt.ylim([0, 1])
    predicted_label = np.argmax(predictions_array)

    thisplot[predicted_label].set_color('red')
    thisplot[true_label].set_color('blue')

x_train, x_test, y_train, y_test = train_test_split(data_x, data_y, test_size=0.1)

reconstructed_model = keras.models.load_model("/home/laptop/codes/modelotest.pb")
image_x = np.array(x_train[1])
image = np.resize(image_x,(1,72,128,1))

imaget_x = np.array(x_test[0])
imaget = np.resize(imaget_x,(1,72,128,1))
res=reconstructed_model.predict(imaget)
print(imaget.shape)
print(imaget.dtype)
print(res)
plt.imshow(np.reshape(data_x[1],(72,128),order='F'))
print("imprimeeeeeee")
plt.show()
