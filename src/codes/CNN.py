# -*- coding: utf-8 -*-
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

data_x = load('/home/laptop/codes/data_x3.npy')
data_y = load('/home/laptop/codes/data_y3.npy')

x_train, x_test, y_train, y_test = train_test_split(data_x, data_y, test_size=0.1)


image_x = np.array(x_train)
print(image_x.shape)

image = np.reshape(image_x,(900,72,128,1),order='F')

imaget_x = np.array(x_test)
print(imaget_x.shape)
imaget = np.reshape(imaget_x,(100,72,128,1),order='F')

modelalex = tf.keras.models.Sequential([
    tf.keras.layers.Conv2D(filters=48, kernel_size=(3, 3), activation=tf.nn.relu, input_shape=(72,128, 1), padding='same'),
    tf.keras.layers.MaxPooling2D(pool_size=(3, 3)),
    tf.keras.layers.Conv2D(filters=128, kernel_size=(3, 3), activation=tf.nn.relu, padding='same'),
    tf.keras.layers.MaxPooling2D(pool_size=(3, 3)),
    tf.keras.layers.Conv2D(filters=192, kernel_size=(3, 3), activation=tf.nn.relu, padding='same'),
    tf.keras.layers.Conv2D(filters=192, kernel_size=(3, 3), activation=tf.nn.relu, padding='same'),
    tf.keras.layers.Conv2D(filters=128, kernel_size=(3, 3), activation=tf.nn.relu, padding='same'),
    tf.keras.layers.MaxPooling2D(pool_size=(3, 3)),
])

modelalex.add(tf.keras.layers.Flatten())  # Flatten "squeezes" a 3-D volume down into a single vector.
modelalex.add(tf.keras.layers.Dense(1024, activation=tf.nn.relu))
modelalex.add(tf.keras.layers.Dropout(rate=0.2))
modelalex.add(tf.keras.layers.Dense(1024, activation=tf.nn.relu))
modelalex.add(tf.keras.layers.Dense(5, activation=tf.nn.softmax))

modelalex.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])
tic = time.time()
historyalex=modelalex.fit(image, y_train,validation_data=(imaget, y_test),batch_size=None, epochs=30, verbose=2)
tic = time.time() - tic

modelalex.save("/home/laptop/codes/modelotest.pb")
#reconstructed_model = keras.models.load_model("/home/laptop/codes/modelotestAtom.pb")

#reconstructed_model.summary()
