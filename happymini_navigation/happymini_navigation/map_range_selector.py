import cv2
import sys
import numpy as np

class MapSelector:
    def __init__(self, path,  vertex=4, output_path="arena.png", dilate=3, skip_wait=False):
        self.dilate = dilate
        self.ref_img = cv2.imread(path, 1)
        self.ref_img_gray = cv2.imread(path, 0)
        self.output_path = output_path
        self.vertex = int(vertex)
        self.scale = 4
        self.img = cv2.resize(self.ref_img, None, None, self.scale, self.scale)
        self.registered_points = []
        self.registered_points_vis = []
        cv2.imshow('map', self.img)
        cv2.setMouseCallback('map', self.click_event)

    
    def click_event(self, event, x, y, flags, params):
  
        # checking for left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:
  
            # displaying the coordinates
            # on the Shellx
            print(x, ' ', y)

            x_pt = round(x / self.scale)
            y_pt = round(y / self.scale)
            self.registered_points.append((x_pt, y_pt))
            self.registered_points_vis.append((x, y))
            cv2.circle(self.img, (x,y), 5, (0,0,255), 5)
            
            if len(self.registered_points) > 1:
                idx = len(self.registered_points) -1
                cv2.line(self.img, self.registered_points_vis[idx-1],self.registered_points_vis[idx],(0,0,255),3)

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.img, str(x_pt) + ',' +
                        str(y_pt), (x,y), font,
                        1, (255, 0, 0), 2)
            cv2.imshow('map', self.img)


            if len(self.registered_points) == self.vertex:
                cv2.line(self.img, self.registered_points_vis[0],self.registered_points_vis[self.vertex-1],(0,0,255),3)
                gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
                contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                points_np = np.asarray(self.registered_points_vis)
                points_np_map = np.asarray(self.registered_points)
                map_mask = np.zeros_like(self.ref_img_gray)
                cv2.fillPoly(map_mask, pts = [points_np_map], color=255)
                cv2.fillPoly(self.img, pts =[points_np], color=(0,0,255))
                kernel = np.ones((self.dilate, self.dilate), np.uint8)
                dil = cv2.dilate(map_mask, kernel,iterations=1)
                cv2.imshow("map", self.img)
                cv2.waitKey(1000)
                cv2.imshow("output", map_mask)
                cv2.imwrite(self.output_path, map_mask)
                cv2.imwrite(self.output_path.replace(".png", "-big.png"), dil)
                cv2.waitKey(1000)
                sys.exit(0)
                
  

if __name__=="__main__":
    if len(sys.argv) == 4:
        mapselect = MapSelector(sys.argv[1], sys.argv[2], sys.argv[3])
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    elif len(sys.argv) ==5:
        print("Usage : python3 map_range_selector.py [map path] [vertices] [output path] [expand size] [map resolution] ")
    
    elif len(sys.argv) == 6:
        dilationsize = int(float(sys.argv[4])/float(sys.argv[5]))
        mapselect = MapSelector(sys.argv[1], sys.argv[2], sys.argv[3], dilate=dilationsize)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Usage : python3 map_range_selector.py [map path] [vertices] [output path]")
