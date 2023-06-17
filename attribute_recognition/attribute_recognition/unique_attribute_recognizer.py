import torch
import numpy as np
import cv2
from PIL import Image as PILImage
from deepface import DeepFace
from transformers import AutoProcessor, AutoModelForCausalLM
import nltk
import os

class UniqueAttributeRecognizer:

    def __init__(self):
        self.__facedetect_backends = [
            'opencv', 
            'ssd', 
            'dlib', 
            'mtcnn', 
            'retinaface', 
            'mediapipe'
        ]

        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        #self.processor = AutoProcessor.from_pretrained("microsoft/git-base-coco")
        #self.model = AutoModelForCausalLM.from_pretrained("microsoft/git-base-coco")
        self.model = AutoModelForCausalLM.from_pretrained("microsoft/git-large-r-textcaps")         
        self.model.to(self.device)
        self.processor = AutoProcessor.from_pretrained("microsoft/git-large-r-textcaps")
        


    def recognizeAttributes(self, image : np.ndarray, env_img : np.ndarray) -> str:
        """
        """
        print("received requests")
        pil_img = PILImage.fromarray(env_img).convert("RGB")

        demographics = self.recognizeFace(image)
        pixel_values = self.processor(images=pil_img, return_tensors="pt").to(self.device).pixel_values    
        generated_ids = self.model.generate(pixel_values=pixel_values, max_length=50)
        generated_caption = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
        #generated_caption = ""

        sentence = self.attributesToSentence(generated_caption, demographics)
        return sentence

    def attributesToSentence(self, description : str, demographics : dict) -> str:
        """
        convert dict attributes information to natural sentence.
        Returns:
        - str : describes information about the attributes.
        """
        man_words = ["male", "man", "boy"]
        woman_words = ["woman", "women", "lady", "girl", "female"]

        gender_sentence = ""
        pronoun = {"man" : "he", "woman" :  "she"}
        gender_words = {"man" : "male", "woman" : "female"}

        # if face cannot be found, gender detection would be done by GiT
        if not demographics["face_found"]:
            for i in range(len(man_words)):
                if description.find(man_words[i])  != -1:
                    demographics["dominant_gender"] = "Man"
                    break
            for i in range(len(woman_words)):
                if description.find(woman_words[i]) != -1:
                    demographics["dominant_gender"] = "Woman"
                    break
            # default strat : man
            if not "dominant_gender" in demographics:
                demographics["dominant_gender"] = "Man"


        pron = pronoun[demographics["dominant_gender"].lower()]
        gender_sentence = "{0} is {1}.".format(pron, gender_words[demographics["dominant_gender"].lower()])
        
        young_words = ["boy", "child", "girl", "teenager", "baby", "infant", "junior"]
        old_words = ["elderly", "old", "aged", "senior"]
        if not demographics["face_found"]:
            for i in range(len(young_words)):
                if description.find(young_words[i]) != -1:
                    demographics["age"] = 20
                    break
            for i in range(len(old_words)):
                if description.find(old_words[i]) != -1:
                    demographics["age"] = 60
                    break
        # default strat : young
        if not "age" in demographics:
            demographics["age"] = 20

        age_description = "young" if demographics["age"] < 30 else ""
        age_description = "middle" if demographics["age"] >= 30 and demographics["age"] < 60 else age_description
        age_description = "old" if demographics["age"] > 60 else age_description

        age_sentence = "{0} looks {1}.".format(pron, age_description)

        return "{0} {1} {2}".format(description, gender_sentence, age_sentence)






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
