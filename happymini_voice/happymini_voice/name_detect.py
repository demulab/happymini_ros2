import rclpy
from rclpy.node import Node
from happymini_msgs.srv import NameDetect
import nltk                                                                                      
import numpy as np
from nltk.util import ngrams
from nltk.metrics.distance import edit_distance
from nltk.metrics.distance import jaccard_distance
from nltk.cluster.util import cosine_distance

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
        self.word_list = ["amelia", "angel", "ava", "charlie", "charlotte", "hunter", "max", "mia", "olivia", "parker", "sam", "jack", "noah", "thomas", "whilliam"]


    def calculate_similarity(self, srv_req, srv_res):
        target_word = srv_req.text.lower()
        
        print("Target Word:", target_word)
        
        target_vector = np.array([1 if ch in target_word else 0 for ch in sorted(set(target_word))])
        best_word = None
        best_sim = -1
        srv_res.result ='None'
        
        for word in self.word_list:
            word_vector = np.array([1 if ch in word else 0 for ch in sorted(set(target_word))]) # 修正点
            
            levenshtein_dist = edit_distance(target_word, word)
            #jaccard_dist = jaccard_distance(set(ngrams(target_word, 3)), set(ngrams(word, 3)))
            cosine_dist = cosine_distance(target_vector, word_vector)
            
            levenshtein_sim = 100 - (levenshtein_dist / max(len(target_word), len(word))) * 100
            #jaccard_sim = (1 - jaccard_dist) * 100
            cosine_sim = (1 - cosine_dist) * 100
            all_sim = levenshtein_sim * 0.2 + cosine_sim * 0.8
            
            print("Word:", word)
            print("Levenshtein Similarity:", levenshtein_sim)
            #print("Jaccard Similarity:", jaccard_sim)
            print("Cosine Similarity:", cosine_sim)
            print("Similarity:", all_sim)
            print()

            if all_sim > best_sim:
                best_sim = all_sim
                best_word = word
        if best_sim > 70:
            srv_res.result = best_word
        if srv_res.result != 'None':
            self.word_list.remove(best_word)
            print(self.word_list)
        print("Best Word:", srv_res.result)
        print("Similarity:", best_sim)
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
