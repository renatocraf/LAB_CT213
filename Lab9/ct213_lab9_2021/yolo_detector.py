from tensorflow.keras.models import load_model
import cv2
import numpy as np
from utils import sigmoid


class YoloDetector:
    """
    Represents an object detector for robot soccer based on the YOLO algorithm.
    """
    def __init__(self, model_name, anchor_box_ball=(5, 5), anchor_box_post=(2, 5)):
        """
        Constructs an object detector for robot soccer based on the YOLO algorithm.

        :param model_name: name of the neural network model which will be loaded.
        :type model_name: str.
        :param anchor_box_ball: dimensions of the anchor box used for the ball.
        :type anchor_box_ball: bidimensional tuple.
        :param anchor_box_post: dimensions of the anchor box used for the goal post.
        :type anchor_box_post: bidimensional tuple.
        """
        self.network = load_model(model_name + '.hdf5')
        self.network.summary()  # prints the neural network summary
        self.anchor_box_ball = anchor_box_ball
        self.anchor_box_post = anchor_box_post

    def detect(self, image):
        """
        Detects robot soccer's objects given the robot's camera image.

        :param image: image from the robot camera in 640x480 resolution and RGB color space.
        :type image: OpenCV's image.
        :return: (ball_detection, post1_detection, post2_detection), where each detection is given
                by a 5-dimensional tuple: (probability, x, y, width, height).
        :rtype: 3-dimensional tuple of 5-dimensional tuples.
        """

        image = self.preprocess_image(image)
        output = self.network.predict(image)
        ball_detection, post1_detection, post2_detection = self.process_yolo_output(output)

        return ball_detection, post1_detection, post2_detection

    def preprocess_image(self, image):
        """
        Preprocesses the camera image to adapt it to the neural network.

        :param image: image from the robot camera in 640x480 resolution and RGB color space.
        :type image: OpenCV's image.
        :return: image suitable for use in the neural network.
        :rtype: NumPy 4-dimensional array with dimensions (1, 120, 160, 3).
        """
        # Todo: implement image preprocessing logic
        image = cv2.resize(image, (160, 120), interpolation=cv2.INTER_AREA)
        image = np.array(image)
        image = image/255.0
        image = np.reshape(image, (1, 120, 160, 3))
        return image

    def process_yolo_output(self, output):
        """
        Processes the neural network's output to yield the detections.

        :param output: neural network's output.
        :type output: NumPy 4-dimensional array with dimensions (1, 15, 20, 10).
        :return: (ball_detection, post1_detection, post2_detection), where each detection is given
                by a 5-dimensional tuple: (probability, x, y, width, height).
        :rtype: 3-dimensional tuple of 5-dimensional tuples.
        """
        coord_scale = 4 * 8  # coordinate scale used for computing the x and y coordinates of the BB's center
        bb_scale = 640  # bounding box scale used for computing width and height
        output = np.reshape(output, (15, 20, 10))  # reshaping to remove the first dimension

        prob = [0, 0, 0]
        coord_bola = np.array([0, 0])
        coord_trave1 = np.array([0, 0])
        coord_trave2 = np.array([0, 0])

        sigm = sigmoid(output)

        for i in range(15):
            for j in range(20):
                #observando bola
                if(sigm[i][j][0] > prob[0]):
                    prob[0] = sigm[i][j][0]
                    coord_bola = [i, j]
                #observando trave1
                if (sigm[i][j][5] > prob[1]):
                    prob[2] = float(prob[1])
                    coord_trave2 = [int(coord_trave1[0]), int(coord_trave1[1])]
                    prob[1] = sigm[i][j][5]
                    coord_trave1 = [i, j]
                #observando trave2
                elif (sigm[i][j][5] > prob[2]):
                    prob[2] = sigm[i][j][5]
                    coord_trave2 = [i, j]

        ball = np.array([0, 0, 0, 0])
        # coordenadas bolas
        ball[0] = (coord_bola[1]+sigm[coord_bola[0]][coord_bola[1]][1])*coord_scale
        ball[1] = (coord_bola[0]+sigm[coord_bola[0]][coord_bola[1]][2])*coord_scale
        #dimensoes da bola
        ball[2] = bb_scale * self.anchor_box_ball[0] * np.exp(output[coord_bola[0]][coord_bola[1]][3])
        ball[3] = bb_scale * self.anchor_box_ball[1] * np.exp(output[coord_bola[0]][coord_bola[1]][4])

        trave1 = np.array([0, 0, 0, 0])
        # coordenadas trave1
        trave1[0] = (coord_trave1[1] + sigm[coord_trave1[0]][coord_trave1[1]][6]) * coord_scale
        trave1[1] = (coord_trave1[0] + sigm[coord_trave1[0]][coord_trave1[1]][7]) * coord_scale
        # dimensoes da bola
        trave1[2] = bb_scale * self.anchor_box_post[0] * np.exp(output[coord_trave1[0]][coord_trave1[1]][8])
        trave1[3] = bb_scale * self.anchor_box_post[1] * np.exp(output[coord_trave1[0]][coord_trave1[1]][9])

        trave2 = np.array([0, 0, 0, 0])
        # coordenadas trave1
        trave2[0] = (coord_trave2[1] + sigm[coord_trave2[0]][coord_trave2[1]][6]) * coord_scale
        trave2[1] = (coord_trave2[0] + sigm[coord_trave2[0]][coord_trave2[1]][7]) * coord_scale
        # dimensoes da bola
        trave2[2] = bb_scale * self.anchor_box_post[0] * np.exp(output[coord_trave2[0]][coord_trave2[1]][8])
        trave2[3] = bb_scale * self.anchor_box_post[1] * np.exp(output[coord_trave2[0]][coord_trave2[1]][9])

        ball_detection = (prob[0], ball[0], ball[1], ball[2], ball[3])
        post1_detection = (prob[1], trave1[0], trave1[1], trave1[2], trave1[3])
        post2_detection = (prob[2], trave2[0], trave2[1], trave2[2], trave2[3])


        return ball_detection, post1_detection, post2_detection
