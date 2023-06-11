import tensorflow as tf
from .deepmar import DeepMAR
import numpy as np
import cv2
from PIL import Image as PILImage
from deepface import DeepFace
from transformers import AutoProcessor, AutoModelForCausalLM
from .attributeinfo import AttributeInfo
        

class AttributeRecognizer:
    def __init__(self, weights_path : str ="out/deepmar"):
        """
        Initializes an AttributeRecognizer object.

        Parameters:
        - weights_path (str): The path to the weights file for the DeepMAR model.
        """
        self.__model = DeepMAR(35)
        self.__model.load_weights(weights_path)
        self.__attrInfo = AttributeInfo()
        self.__facedetect_backends = [
            'opencv', 
            'ssd', 
            'dlib', 
            'mtcnn', 
            'retinaface', 
            'mediapipe'
        ]
        self.processor = AutoProcessor.from_pretrained("microsoft/git-large-r-textcaps")
        self.model = AutoModelForCausalLM.from_pretrained("microsoft/git-large-r-textcaps")        


    def preprocessROSImage(self, image :np.ndarray):
        """
        Preprocesses an image to RGB format for ROS.

        Parameters:
        - image (np.ndarray): The input image in BGR format.

        Returns:
        - np.ndarray: The preprocessed image.
        """
        rescaled_image = cv2.resize(image, (224, 224))
        rgb_image = cv2.cvtColor(rescaled_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(rgb_image)
        
        input_image = tf.keras.preprocessing.image.img_to_array(pil_image)
        input_image = tf.keras.applications.resnet.preprocess_input(input_image)
        input_image = tf.expand_dims(input_image, axis=0)        
        return input_image


    def preprocessImage(self, image_path : str):
        """
        Preprocesses an image from the given file path.

        Parameters:
        - image_path (str): The path to the image file.

        Returns:
        - np.ndarray: The preprocessed image.
        """
        image = tf.keras.preprocessing.image.load_img(image_path, target_size=(224, 224))
        print("image type")
        print(image)
        image = tf.keras.preprocessing.image.img_to_array(image)
        print(image.shape)
        print(image.dtype)
        image = tf.keras.applications.resnet.preprocess_input(image)
        image = tf.expand_dims(image, axis=0) # Add batch dimension
        return image
    
    def recognizeAttributesWithPath(self, image_path : str) -> str:
        """
        Recognizes attributes in an image and returns the description.

        Returns:
        - str: The description of recognized attributes.
        """
        image = self.preprocessImage(image_path)
        predictions = self.__model.predict(image)
        attrs = np.nonzero(predictions >= 0)[1]
        print(predictions)
        print(attrs)
        self.__attrInfo.storeAttributes(attrs)

        result = "{0}.\n{1}.\n{2}.\n{3}.\n".format(self.__attrInfo.describeGender(), self.__attrInfo.describeAge(), self.__attrInfo.describeFasionStyle(), self.__attrInfo.describeClothDetails())
        return result

    def recognizeAttributesNumpy(self, image  : np.ndarray) -> str:
        """
        Recognizes attributes in an image in numpy format and returns the description.

        Returns:
        - str: The description of recognized attributes.
        """
        image_input = self.preprocessROSImage(image)
        predictions = self.__model.predict(image_input)
        attrs = np.nonzero(predictions >= 0)[1]

        demographies = self.recognizeFace(image)
        #for i in range(len(self.__attrInfo.label)):
        #    print("{0}: {1}".format(self.__attrInfo.label[i], predictions[0][i]))
        #print(attrs)
        self.__attrInfo.storeAttributes(attrs, demographies)



        result = "{0}.\n{1}.\n{2}.\n{3}.\n".format(self.__attrInfo.describeGender(), self.__attrInfo.describeAge(), self.__attrInfo.describeFasionStyle(), self.__attrInfo.describeClothDetails())


        
        #result = "{0}.\n{1}.\n{2}.\n.\n".format(self.__attrInfo.describeGender(), self.__attrInfo.describeFasionStyle(), self.__attrInfo.describeClothDetails())
        return result
    
    def recognizeFace(self, image : np.ndarray) -> dict:
        """
        Recognize demographic feature based on face. This method was made because the attribute detection for the gender and age were bad when using DeepMAR.
        Returns:
        - dict : Demographic information
        """
        try:
            #facial analysis
            demographies = DeepFace.analyze(image, 
            detector_backend = self.__facedetect_backends[3]
            )
            result = demographies[0]
            result["face_found"] = True
            return result

        except ValueError:
            result = dict()
            result["face_found"] = False
            return result





if __name__ == "__main__":
    attributeRecognizer = AttributeRecognizer()
    result =  attributeRecognizer.recognizeAttributesWithPath("categories/50cm.png")
    print(result)
    

    
