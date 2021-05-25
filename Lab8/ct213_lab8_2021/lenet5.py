from tensorflow.keras import layers, activations
from tensorflow.keras.models import Sequential


def make_lenet5():
    model = Sequential()


    # Todo: implement LeNet-5 model
    # entrada
    model.add(layers.Conv2D(filters=1,
                            kernel_size=(1,1),
                            strides=(1,1),
                            input_shape=(32, 32, 1)))
    # camada 1
    model.add(layers.Conv2D(filters=6,
                            kernel_size=(5, 5),
                            strides=(1, 1),
                            activation=activations.tanh))
    # camada 2
    model.add(layers.AveragePooling2D(pool_size=(2, 2),
                                      strides=(2, 2)))
    # camada 3
    model.add(layers.Conv2D(filters=16,
                            kernel_size=(5, 5),
                            strides=(1, 1),
                            activation=activations.tanh))
    # camada 4
    model.add(layers.AveragePooling2D(pool_size=(2, 2),
                                      strides=(2, 2)))
    # camada 5
    model.add(layers.Conv2D(filters=120,
                            kernel_size=(5, 5),
                            strides=(1, 1),
                            activation=activations.tanh))

    # camada 6
    model.add(layers.Flatten())
    model.add(layers.Dense(units=84,
                           activation=activations.tanh))
    # camada 7
    model.add(layers.Dense(units=10,
                           activation=activations.softmax))



    return model
