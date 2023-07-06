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
        self.lang_sam = LangSAM()
        self.model.to(self.device)
        self.processor = AutoProcessor.from_pretrained("microsoft/git-large-r-textcaps")


    def recognizeClothColor(self, image :np.ndarray) -> str:

        lower = ["shorts", "jeans", "skirt", "trouser", "leg"]
        #shoe, sandal, boot
        whole = ["suits", "dress"]
        shoe = ["sandal", "boot", "shoe"]

        lower_blacklist = ["leg"]
        shoe_nocolorlist = ["sandal"]



        pil_img = PILImage.fromarray(image).convert("RGB")

        lower_bb = []
        lower_idx = -1
        shoe_bb = []
        shoe_idx = -1        
        tops_bb = []
        whole_bb = []
        whole_idx = -1

        for i in range(len(lower)):
            boxes, logits, phrases = self.lang_sam.predict_dino(pil_img, lower[i], 0.5, 0.25)
            if len(boxes.numpy()) > 0:
                lower_bb = np.int16(boxes.numpy())
                x1 = lower_bb[0][0]
                y1 = lower_bb[0][1]
                x2 = lower_bb[0][2]
                y2 = lower_bb[0][3]

                img_area = image.shape[0] * image.shape[1]
                area = (x2-x1) * (y2-y1)
                area_r = float(area/img_area)
                if area_r > 0.7:
                    lower_bb = []
                    continue

                tops_bb = [0, 0, image.shape[1]- 1, lower_bb[0][1]] 
                lower_idx = i
                break
        """
        for i in range(len(whole)):
            boxes, logits, phrases = self.lang_sam.predict_dino(pil_img, whole[i], 0.5, 0.25)          
            if len(boxes.numpy()) > 0:
                whole_bb = np.int16(boxes.numpy())
                whole_idx = i
                break  
        """
        for i in range(len(shoe)):
            boxes, logits, phrases = self.lang_sam.predict_dino(pil_img, shoe[i], 0.5, 0.25)          
            if len(boxes.numpy()) > 0:
                shoe_bb = np.int16(boxes.numpy())
                shoe_idx = i
                break  

        #shoe_boxes, logits, phrases = self.lang_sam.predict_dino(pil_img, "shoe", 0.5, 0.25)
        #if len(shoe_boxes.numpy()) > 0:
        #    shoe_bb = np.uint16(shoe_boxes.numpy())

        lower_color = ""
        shoe_color = ""
        tops_color = ""

        for i in range(len(lower_bb)):
            x1 = lower_bb[i][0]
            y1 = lower_bb[i][1]
            x2 = lower_bb[i][2]
            y2 = lower_bb[i][3]
            target_img = image[y1:y2, x1:x2]
            cv2.imwrite(f"/home/demulab/test_data/{lower[lower_idx]}.png", target_img)
            color_row = np.median(target_img, axis=0)
            color = np.median(color_row, axis=0)
            lower_color = self.findNearestColor(color[2], color[1], color[0])

        for i in range(len(shoe_bb)):
            x1 = shoe_bb[i][0]
            y1 = shoe_bb[i][1]
            x2 = shoe_bb[i][2]
            y2 = shoe_bb[i][3]

            target_img = image[y1:y2, x1:x2]

            cv2.imwrite(f"/home/demulab/test_data/{shoe[shoe_idx]}.png", target_img)
            color_row = np.median(target_img, axis=0)
            color = np.median(color_row, axis=0)
            shoe_color = self.findNearestColor(color[2], color[1], color[0], black_penalty=10)

        if len(whole_bb) > 0:
            x1 = whole_bb[0][0]
            y1 = whole_bb[0][1]
            x2 = whole_bb[0][2]
            y2 = whole_bb[0][3]

            target_img = image[y1:y2, x1:x2]
            cv2.imwrite(f"/home/demulab/test_data/{whole[whole_idx]}.png", target_img)


        if len(lower_bb) > 0:
            x1 = tops_bb[0]
            y1 = tops_bb[1]
            x2 = tops_bb[2]
            y2 = tops_bb[3]

            target_img = image[y1:y2, x1:x2]

            color_row = np.median(target_img, axis=0)
            color = np.median(color_row, axis=0)
            tops_color = self.findNearestColor(color[2], color[1], color[0])


        lower_name = "bottoms"
        shoe_name = "shoe"
        whole_name = ""


        if lower_idx >= 0:
            lower_name = lower[lower_idx]
            if lower_name in lower_blacklist:
                lower_name = "bottoms"
        if shoe_idx >= 0:
            shoe_name = shoe[shoe_idx]
            if shoe_name in shoe_nocolorlist:
                shoe_color = ""

        if whole_idx >= 0 and lower_name == "bottoms":
            whole_name = whole[whole_idx]
            lower_color = ""
            lower_name = ""

        color_info = {"bottoms" : lower_color, "shoe" : shoe_color}
        cloth_info = {"bottoms" : lower_name, "shoe" : shoe_name, "whole" : whole_name}
        wear_info = {"color" : color_info, "cloth" : cloth_info}

        return wear_info
        

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

        cloth_info = self.recognizeClothColor(image)
        #generated_caption = ""
        sentence = self.getFeatureTwins(generated_caption, demographics, cloth_info, self.feature_count)
        #sentence = self.attributesToSentence(generated_caption, demographics, cloth_info)
        self.feature_count += 1
        return sentence

    def getFeatureTwins(self, description, demographics, cloth_info, mode):
        combos = [["gender", "age"], ["bottoms_type", "bottoms_color"], ["shoe_color", "tops_color"]]
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


        if mode == 0:
            gender = demographics["dominant_gender"].lower()
            age = "young" if demographics["age"] < 35  else "middle"
            feature_sentence = f"The person is {age} {gender}."

        elif mode == 1:
            bottoms_type = cloth_info["cloth"]["bottoms"]
            bottoms_color = cloth_info["color"]["bottoms"]
            feature_sentence = f"The person is wearing {bottoms_color} {bottoms_type}."

        elif mode == 2:
            shoe_color = cloth_info["color"]["shoe"]
            

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
            feature_sentence = f"The person is wearing {tops_color} shirt and {shoe_color} shoes."


        return feature_sentence


        



    def attributesToSentence(self, description : str, demographics : dict, cloth_info  :dict) -> str:
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

        age_description = "young" if demographics["age"] < 35 else ""
        age_description = "middle" if demographics["age"] >= 35 and demographics["age"] < 60 else age_description
        age_description = "old" if demographics["age"] > 60 else age_description

        age_sentence = "{0} looks {1}.".format(pron, age_description)

        race_sentence = ""
        if demographics["face_found"]:
            race_sentence = "{0} is {1}.".format(pron, demographics["dominant_race"])
        
        color_sentence = ""

        color_info = cloth_info["color"]

        if color_info["shoe"] is not "" or color_info["bottoms"] is not "":
            color_sentence = "{0} is wearing a {1} {2} and a {3} {4}.".format(pron, color_info["shoe"], cloth_info["cloth"]["shoe"], color_info["bottoms"], cloth_info["cloth"]["bottoms"])

        if cloth_info["cloth"]["whole"] is not "":
            color_sentence = "{0} wears a {1} and a {2} {3}.".format(pron, cloth_info["cloth"]["whole"], color_info["shoe"], cloth_info["cloth"]["shoe"])


        return "{0}_{1}_{2}_{3}".format(description, gender_sentence, age_sentence, color_sentence)






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
