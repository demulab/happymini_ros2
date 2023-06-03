from enum import Enum
import numpy as np

class Age(Enum):
    U30 = 0,
    U45 = 1,
    U60 = 2,
    O60 = 3,
    UNKNOWN = 5

class FasionStyle(Enum):
    CASUAL = 0,
    FORMAL = 1,
    UNKNOWN = 2

class TopsPattern(Enum):
    LOGO = 0,
    PLAID = 1,
    THIN_STRIPES = 2,
    PLAIN = 3

class TopsType(Enum):
    JACKET = 0,
    TSHIRT = 1,
    VNECK = 2,
    OTHERS = 3

class BottomsType(Enum):
    JEANS = 0,
    SHORTS = 1,
    SHORTSKIRT = 2,
    TROUSERS = 3,
    UNKNOWN = 4

class Accessories(Enum):
    HAT = 0,
    MUFFLER = 1,
    SUNGLASSES = 2,
    NOTHING = 3,
    UNKNOWN = 4

class FootwearType(Enum):
    LEATHER_SHOES = 0,
    SANDALS = 1,
    SHOES = 2,
    SNEAKER = 3,
    UNKNOWN = 4

class Carry(Enum):
    BACKPACK = 0,
    MESSENGER_BAG = 1,
    PLASTIC_BAG = 2,
    NOTHING = 3,
    OTHERS = 4

class AttributeInfo:
    def __init__(self):


        ## boolean attributes
        self.male = False
        self.hair_long = False
        self.short_sleeve = False

        ## age attributess
        self.age = Age.UNKNOWN

        ## fasion attributes
        self.tops_fasion = FasionStyle.UNKNOWN
        self.bottoms_fasion = FasionStyle.UNKNOWN

        ## tops attributes
        self.tops_pattern = TopsPattern.PLAIN
        self.tops_type = TopsType.OTHERS

        ## bottoms attributes
        self.bottoms_type = BottomsType.UNKNOWN
        self.footwear = FootwearType.UNKNOWN

        ## accessories attributes
        self.carry = Carry.OTHERS
        self.accessories = Accessories.UNKNOWN


        ## boolean attributes labels
        self.genderattr_label = {"male" : 16}
        self.hairstyle_label = {"long" : 15}
        self.short_sleeve_label = {"sleeve" : 26}

        self.ageattr_label = {"U30" : 0, "U45" : 1, "U60" : 2, "O60" : 3}
        self.fasionstyle_label = {"lower_casual" : 6, "upper_casual" :7, "lower_formal" : 8, "upper_formal" : 9}
        self.carry_label = {"backpack" : 4, "others" : 5, "messenger_bag" :  17, "nothing" : 20, "plastic_bag" : 22}
        self.accessories_label = {"hat" : 10, "muffler" : 18, "nothing" : 19, "sunglasses" : 30}


        self.upperbody_cloth_label = {"jacket" : 11, "tshirt" : 32, "others": 33, "vneck" : 34}
        self.tops_pattern_label = {"logo" : 14, "plaid" : 21, "thin_stripes" : 29}
        self.lowerbody_cloth_label = {"jeans" : 12, "shorts" : 25, "shortskirt" : 27, "trousers" : 31}
        self.footwear_label = {"leather_shoes" : 13, "sandals" : 23, "shoes" : 24, "sneaker" : 28}

    def storeAge(self, data : np.ndarray) -> None:
        if self.ageattr_label["U30"] in data:
            self.age = Age.U30
        elif self.ageattr_label["U45"] in data:
            self.age = Age.U45
        elif self.ageattr_label["U60"] in data:
            self.age = Age.U60
        elif self.ageattr_label["O60"] in data:
            self.age = Age.O60
        else:
            self.age = Age.UNKNOWN

    def storeFasionStyle(self, data : np.ndarray) -> None:
        if self.fasionstyle_label["upper_casual"] in data:
            self.tops_fasion = FasionStyle.CASUAL
        elif self.fasionstyle_label["upper_formal"] in data:
            self.tops_fasion = FasionStyle.FORMAL
        else:
            self.tops_fasion = FasionStyle.UNKNOWN
        
        if self.fasionstyle_label["lower_casual"] in data:
            self.bottoms_fasion = FasionStyle.CASUAL
        elif self.fasionstyle_label["lower_formal"] in data:
            self.bottoms_fasion = FasionStyle.FORMAL
        else:
            self.bottoms_fasion = FasionStyle.UNKNOWN

    def storeTopsPattern(self, data : np.ndarray) -> None:
        if self.tops_pattern_label["logo"] in data:
            self.tops_pattern = TopsPattern.LOGO
        elif self.tops_pattern_label["plaid"] in data:
            self.tops_pattern = TopsPattern.PLAID
        elif self.tops_pattern_label["thin_stripes"] in data:
            self.tops_pattern = TopsPattern.THIN_STRIPES
        else:
            self.tops_pattern = TopsPattern.PLAIN

    def storeTopsType(self, data : np.ndarray) -> None:
        if self.upperbody_cloth_label["jacket"] in data:
            self.tops_type = TopsType.JACKET
        elif self.upperbody_cloth_label["tshirt"] in data:
            self.tops_type = TopsType.TSHIRT
        elif self.upperbody_cloth_label["vneck"] in data:
            self.tops_type = TopsType.VNECK
        elif self.upperbody_cloth_label["others"] in data:
            self.tops_type = TopsType.OTHERS
        else:
            self.tops_type = TopsType.OTHERS

    def storeBottomsType(self, data : np.ndarray) -> None:
        if self.lowerbody_cloth_label["jeans"] in data:
            self.bottoms_type = BottomsType.JEANS
        elif self.lowerbody_cloth_label["shorts"] in data:
            self.bottoms_type = BottomsType.SHORTS
        elif self.lowerbody_cloth_label["shortskirt"] in data:
            self.bottoms_type = BottomsType.SHORTSKIRT
        elif self.lowerbody_cloth_label["trousers"] in data:
            self.bottoms_type = BottomsType.TROUSERS
        else:
            self.bottoms_type = BottomsType.UNKNOWN


    def storeFootwear(self, data: np.ndarray) -> None:
        if self.footwear_label["leather_shoes"] in data:
            self.footwear = FootwearType.LEATHER_SHOES
        elif self.footwear_label["sandals"] in data:
            self.footwear = FootwearType.SANDALS
        elif self.footwear_label["shoes"] in data:
            self.footwear = FootwearType.SHOES
        elif self.footwear_label["sneaker"] in data:
            self.footwear = FootwearType.SNEAKER
        else:
            self.footwear = FootwearType.UNKNOWN

    def storeCarry(self, data: np.ndarray) -> None:
        if self.carry_label["backpack"] in data:
            self.carry = Carry.BACKPACK
        elif self.carry_label["messenger_bag"] in data:
            self.carry = Carry.MESSENGER_BAG
        elif self.carry_label["plastic_bag"] in data:
            self.carry = Carry.PLASTIC_BAG
        elif self.carry_label["nothing"] in data:
            self.carry = Carry.NOTHING
        elif self.carry_label["others"] in data:
            self.carry = Carry.OTHERS
        else:
            self.carry = Carry.OTHERS


    def storeAccessories(self, data: np.ndarray) -> None:
        if self.accessories_label["hat"] in data:
            self.accessories = Accessories.HAT
        elif self.accessories_label["muffler"] in data:
            self.accessories = Accessories.MUFFLER
        elif self.accessories_label["sunglasses"] in data:
            self.accessories = Accessories.SUNGLASSES
        elif self.accessories_label["nothing"] in data:
            self.accessories = Accessories.NOTHING
        else:
            self.accessories = Accessories.UNKNOWN

    def storeAttributes(self, data : np.ndarray) -> None:
        
        self.male = self.genderattr_label["male"] in data
        self.hair_long = self.hairstyle_label["long"] in data
        self.short_sleeve = self.short_sleeve_label["sleeve"] in data

        self.storeAge(data)
        self.storeFasionStyle(data)
        self.storeTopsPattern(data)
        self.storeTopsType(data)
        self.storeBottomsType(data)
        self.storeFootwear(data)
        self.storeCarry(data)
        self.storeAccessories(data)
    
    def describeAge(self) -> str:
        pronoun = "He" if self.male else "She"

        age_description = ""

        if self.age == Age.U30:
            age_description = "under 30 years old"
        elif self.age == Age.U45:
            age_description = "over 30 years old, and under 45 years old"
        elif self.age == Age.U60:
            age_description = "over 45 years old, and under 60 years old"
        else:
            age_description = "over 60 years old"
        
        age_sentence = "{0} is {1}".format(pronoun, age_description)
        return age_sentence
    
    def describeGender(self) ->  str:
        pronoun = "He" if self.male else "She"
        gender_description = "male" if self.male else "female"

        gender_sentence = "{0} is {1}".format(pronoun, gender_description)
        return gender_sentence
    
    def describeFasionStyle(self) -> str:
        pronoun = "His" if self.male else "Her"
        fasion_description = ""

        tops_fasion = "casual" if self.tops_fasion == FasionStyle.CASUAL else "formal"
        bottoms_fasion = "casual" if self.bottoms_fasion == FasionStyle.CASUAL else "formal"

        fasionstyle_sentence = "{0} fashion style is, tops {1}, and bottoms {2}".format(pronoun, tops_fasion, bottoms_fasion)
        return fasionstyle_sentence
    
    def describeClothDetails(self) -> str:
        pronoun = "He" if self.male else "She"
        tops_description = ""

        tops_pattern = "plain"
        if self.tops_pattern == TopsPattern.LOGO:
            tops_pattern = "logo"
        elif self.tops_pattern == TopsPattern.PLAID:
            tops_pattern = "plaid"
        elif self.tops_pattern == TopsPattern.THIN_STRIPES:
            tops_pattern = "thin striped"

        tops_cloth = ""
        if self.tops_type == TopsType.JACKET:
            tops_cloth = "jacket"
        elif self.tops_type == TopsType.TSHIRT:
            tops_cloth= "tshirt"
        elif self.tops_type == TopsType.VNECK:
            tops_cloth = "vneck"
        else:
            tops_cloth = "cloth"

        bottoms_cloth = ""
        if self.bottoms_type == BottomsType.JEANS:
            bottoms_cloth = "jeans"
        elif self.bottoms_type == BottomsType.SHORTS:
            bottoms_cloth = "shorts"
        elif self.bottoms_type == BottomsType.SHORTSKIRT:
            bottoms_cloth = "a shortskirt"
        elif self.bottoms_type == BottomsType.TROUSERS:
            bottoms_cloth = "trousers"
        
        bottoms_cloth += ", "

        accessories_description = "with a"
        if self.accessories == Accessories.HAT:
            accessories_description += "hat"
        elif self.accessories == Accessories.MUFFLER:
            accessories_description += "muffler"
        elif self.accessories == Accessories.SUNGLASSES:
            accessories_description += "sunglasses"
        else:
            accessories_description = ""
        
        carry_description = "carrying a "
        if self.carry == Carry.BACKPACK:
            carry_description += "backpack"
        elif self.carry == Carry.MESSENGER_BAG:
            carry_description += "messenger bag"
        elif self.carry == Carry.PLASTIC_BAG:
            carry_description += "plastic bag"
        else:
            carry_description = ""

        accessories_description += ", " if len(carry_description) > 0 else ""


        shoes_description = "and a"
        if self.footwear == FootwearType.LEATHER_SHOES:
            shoes_description += "leather shoes"
        elif self.footwear == FootwearType.SANDALS:
            shoes_description += "sandals"
        elif self.footwear == FootwearType.SNEAKER:
            shoes_description += "sneakers"
        else:
            shoes_description = ""

        carry_description += ", " if len(shoes_description) > 0 else ""

        cloth_sentence = "{0} is wearing a {1} {2}, {3}{4}{5}{6}.".format(pronoun, tops_pattern, tops_cloth, bottoms_cloth, accessories_description, carry_description, shoes_description)
        return cloth_sentence
