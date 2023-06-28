import cv2
import mediapipe as mp
import logging
import numpy as np



class DetectPoseModule():
    def __init__(self):
        self.__mp_drawing_styles = mp.solutions.drawing_styles

        self.__mp_pose_01= mp.solutions.pose
        self.__mp_face_mesh = mp.solutions.face_mesh
        
        self.__mp_drawing = mp.solutions.drawing_utils
        self.__drawing_spec = self.__mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

        self.__mp_pose = mp.solutions.pose.Pose(
          min_detection_confidence=0.1,
          min_tracking_confidence=0.8
        )
        self.__face_mesh = self.__mp_face_mesh.FaceMesh(
                static_image_mode=True,
                max_num_faces=1,
                min_detection_confidence=0.5)

    def detect(self, images):
        pose_result_list = []
        got_face_list = []
        landmark_dict = {}
        image = None
  
        for num, img in enumerate(images):
            color_frame = img
            if color_frame.any():
                image = np.asanyarray(color_frame)
                #self.get_logger().info(f'{image}')
                image.flags.writeable = False
                #image = cv2.cvtColor(image, cv2.COLOE_RGB2BGR)
                pose_results = self.__mp_pose.process(image)
                face_results = self.__face_mesh.process(image)
                image.flags.writeable = True
                if pose_results.pose_landmarks:
                    #self.__mp_drawing.draw_landmarks(image, pose_results.pose_landmarks, self.__mp_pose_01.POSE_CONNECTIONS, landmark_drawing_spec=self.__mp_drawing_styles.get_default_pose_landmarks_style())
                    self.__mp_drawing.draw_landmarks(
                            image,
                            pose_results.pose_landmarks,
                            mp.solutions.pose.POSE_CONNECTIONS,
                            landmark_drawing_spec=self.__mp_drawing_styles.get_default_pose_landmarks_style())
                    for pose_landmarks in pose_results.pose_landmarks.landmark:
                        landmark_dict['nose'] = pose_results.pose_landmarks.landmark[0]
                        landmark_dict['left_ear'] = pose_results.pose_landmarks.landmark[7]
                        landmark_dict['right_ear'] = pose_results.pose_landmarks.landmark[8]
                        landmark_dict['left_shoulder'] = pose_results.pose_landmarks.landmark[11]
                        landmark_dict['right_shoulder'] = pose_results.pose_landmarks.landmark[12]
                        landmark_dict['left_hip'] = pose_results.pose_landmarks.landmark[23]
                        landmark_dict['right_hip'] = pose_results.pose_landmarks.landmark[24]
                if face_results.multi_face_landmarks:
                    for face_landmarks in face_results.multi_face_landmarks:
                        self.__mp_drawing.draw_landmarks(
                                image,
                                face_landmarks,
                                self.__mp_face_mesh.FACEMESH_CONTOURS,
                                self.__drawing_spec,
                                self.__drawing_spec)
                    got_face_list.append(True)
                else:
                    got_face_list.append(False)
                pose_result_list.append(landmark_dict)
                #print(landmark_dict)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                cv2.imshow(f'Image {num}', image)
                cv2.waitKey(1)
        return pose_result_list, got_face_list

def main(args=None):
    module = DetectPoseModule()
