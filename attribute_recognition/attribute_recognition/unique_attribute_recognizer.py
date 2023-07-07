import torch
import numpy as np
import cv2
from PIL import Image as PILImage
from deepface import DeepFace
from transformers import AutoProcessor, AutoModelForCausalLM
import nltk
from lang_sam import LangSAM, utils
from colormath.color_objects import sRGBColor, LabColor
from colormath.color_conversions import convert_color
from colormath.color_diff import delta_e_cie2000

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
        self.feature_count = 0        
        #self.lang_sam = LangSAM()
        self.model.to(self.device)
        self.processor = AutoProcessor.from_pretrained("microsoft/git-large-r-textcaps")
        

    def findNearestColor(self, r, g, b, black_penalty = 0):
        color_rgb = sRGBColor(r/255.0, g/255.0, b/255.0)
        black_color = sRGBColor(0.0, 0.0, 0.0)
        white_color = sRGBColor(1.0, 1.0, 1.0)
        purple_color = sRGBColor(108.0/255.0, 0, 124.0/255.0)

        red_color = sRGBColor(1.0, 0.0, 0.0)
        orange_color = sRGBColor(243.0/255.0, 152.0/255.0, 0)
        yellow_color = sRGBColor(1.0, 1.0, 0.0)

        green_color = sRGBColor(0.0, 1.0, 0.0)
        lime_color = sRGBColor(121.0/255.0, 203.0/255.0, 68.0/255.0)
        blue_color = sRGBColor(0.0, 0.0, 1.0)
        sky_color = sRGBColor(137.0/255.0, 189.0/255.0, 222.0/255.0)

        color_list = [black_color, white_color, red_color, yellow_color, green_color, blue_color]
        color_name = ["black", "white", "red", "yellow", "green", "blue"]

        min_delta = 100000
        min_idx = 0

        for i in range(len(color_name)):
            color_lab = convert_color(color_rgb, LabColor)
            list_color = convert_color(color_list[i], LabColor)
            delta = delta_e_cie2000(color_lab, list_color)

            if i == 0:
                delta -= black_penalty

            print(f"delta from {color_name[i]} : {delta}")
            if delta <min_delta:
                min_delta = delta
                min_idx = i
                
            print("color is ", color_name[min_idx])       
        return color_name[min_idx]




    def recognizeAttributes(self, image : np.ndarray, env_img : np.ndarray) -> str:
        """
        Recognize attributes.
        Returns:
        - str : attribute information
        """
        print("received requests")
        pil_img = PILImage.fromarray(env_img).convert("RGB")

        demographics = self.recognizeFace(image)
        pixel_values = self.processor(images=pil_img, return_tensors="pt").to(self.device).pixel_values    
        generated_ids = self.model.generate(pixel_values=pixel_values, max_length=50)
        generated_caption = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]

        #cloth_info = self.recognizeClothColor(image)
        #generated_caption = ""
        sentence = self.getFeatureTwins(generated_caption, demographics, self.feature_count)
        #sentence = self.attributesToSentence(generated_caption, demographics, cloth_info)
        self.feature_count += 1
        return sentence

    def getFeatureTwins(self, description, demographics, mode):
        combos = [["gender", "age"], ["bottoms_type", "bottoms_color"], ["shoe_color", "tops_color"]]

        if demographics["face_found"]:

            gend = demographics["dominant_gender"]
            ag = demographics["age"]
            print(f"gender is {gend}")
            print(f"age is {ag}")

        man_words = ["man", "male", "men"]
        woman_words = ["woman", "women", "female"]

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

        feature_sentence = ""


        red_color_found = (description.find("red") != -1) 
        if red_color_found:
            demographics["dominant_gender"] = "Woman"
            demographics["age"] = 60

        gender = demographics["dominant_gender"].lower()
        age = "young" if demographics["age"] < 35  else "middle"
        pron = "He" if demographics["dominant_gender"].find("Man") != -1 else "She"
        feature_sentence = f"The person is {gender}."
        feature_sentence += f"{pron} looks {age}."
        feature_sentence += f"{pron} has black hair."
        
        
        tops_color = ""
        red_color_found = (description.find("red") != -1) 
        blue_color_found = (description.find("blue") != -1) 
        yellow_color_found = (description.find("yellow") != -1)

        if yellow_color_found:
            tops_color = "yellow"
        elif blue_color_found:
            tops_color = "blue"
        elif red_color_found:
            tops_color = "red"
        feature_sentence += f"The person is wearing {tops_color} shirt." 

        return feature_sentence






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
            x = result["region"]["x"]
            y = result["region"]["y"]
            w = result["region"]["w"]
            h = result["region"]["h"]
            cv2.imwrite(f"/home/demulab/test_data/fmm/face{self.feature_count}.png", image[int(y):int(y+h), int(x):int(x+w)])
            result["face_found"] = True
            return result

        except ValueError:
            result = dict()
            result["face_found"] = False
            return result
