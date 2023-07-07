import rclpy
from rclpy.node import Node
from happymini_msgs.srv import NameDetect
import nltk                                                                                      
import numpy as np
from nltk.util import ngrams
from nltk.metrics.distance import edit_distance
from nltk.metrics.distance import jaccard_distance
from nltk.cluster.util import cosine_distance
import ngram


class NameDetectServer(Node):
    def __init__(self):
        super().__init__('name_detect')
        #self.declare_parameters(
        #    namespace='',
        #    parameters=[
        #        ('name', rclpy.Parameter.Type.STRING_ARRAY),
        #        ('drinks', None),
        #        ('gender', None),
        #    ])

        self.create_service(NameDetect, 'nd', self.calculate_similarity)
        #self.word_list = self.get_parameter("name")
        #["amelia", "angel", "ava", "charlie", "charlotte", "hunter", "max", "mia", "olivia", "parker", "sam", "jack", "noah", "thomas", "whilliam"]
        self.word_list = ["james", "daniel", "thomas", "alexander", "oliver", "william", "george", "emma", "sophia", "henry", "john"]
        self.drink_list = ["coffee", "lemonade", "tea", "apple juice", "water", "pepsi"]
        #self.word_list = ["amelia", "max", "charlie", "hunter"]
        #self.word_list = ["amelia", "angel", "ava", "charlie", "charlotte", "hunter", "max", "mia", "olivia", "parker", "sam", "jack", "noah", "thomas", "whilliam"]

    def calculate_ngram_similarity(self, word_list : list, word :str, threshold = 0.1) -> list:
        G = ngram.NGram(word_list)
        return G.search(word, threshold)


    def calculate_similarity(self, srv_req, srv_res):
        
        target_word = srv_req.text.lower().replace(".", "").replace(",", "").replace("!", "").replace("?", "")


        if srv_req.type.find("drink") != -1:
            target_word = target_word.replace("my favorite drink is", "").replace("i like", "")

        target_word = target_word.replace("my name is", "").replace(" ", "")


        print("Target Word:", target_word)
        
        target_vector = np.array([1 if ch in target_word else 0 for ch in sorted(set(target_word))])
        best_word = None
        best_sim = -1
        srv_res.result ='None'


        res = self.calculate_ngram_similarity(self.word_list, target_word)
        if srv_req.type.find("drink") !=-1:
            res = self.calculate_ngram_similarity(self.drink_list, target_word)


        print("res", res)
        if len(res) >0:
            if len(res[0]) > 0:
                print("result : ", res[0][0])
                srv_res.result = res[0][0]
        
        #for word in self.word_list:
        #    word_vector = np.array([1 if ch in word else 0 for ch in sorted(set(target_word))]) # 修正点
        #    
        #    levenshtein_dist = edit_distance(target_word, word)
        #    #jaccard_dist = jaccard_distance(set(ngrams(target_word, 3)), set(ngrams(word, 3)))
        #    cosine_dist = cosine_distance(target_vector, word_vector)
        #    
        #    levenshtein_sim = 100 - (levenshtein_dist / max(len(target_word), len(word))) * 100
        #    #jaccard_sim = (1 - jaccard_dist) * 100
        #    cosine_sim = (1 - cosine_dist) * 100
        #    all_sim = levenshtein_sim 
        #    
        #    print("Word:", word)
        #    print("Levenshtein Similarity:", levenshtein_sim)
        #    #print("Jaccard Similarity:", jaccard_sim)
        #    print("Cosine Similarity:", cosine_sim)
        #    print("Similarity:", all_sim)
        #    print()

        #    if all_sim > best_sim:
        #        best_sim = all_sim
        #        best_word = word
        #if best_sim > 70:
        #    srv_res.result = best_word
        if srv_res.result != 'None':
            if srv_req.type.find("drink") != -1:
                self.drink_list.remove(res[0][0])
                print(self.drink_list)
            else:
                self.word_list.remove(res[0][0])
                print(self.word_list)
        #print("Best Word:", srv_res.result)
        #print("Similarity:", best_sim)
        return srv_res


def main():
    rclpy.init()
    nds = NameDetectServer()
    try:
        rclpy.spin(nds)
    except KeyboardInterrupt:
        pass
    nds.destroy_node()
    rclpy.shutdown()
