from PIL import Image
import cv2 
import numpy as np 

vidcap = cv2.VideoCapture('./TeamCode/src/main/python/Red_prop_new.mp4')
width, height = int(vidcap.get(3)), int(vidcap.get(4))
print(width, height)
final_imgs = []
success,image = vidcap.read()
count = 0

def imgrec(img):
    # img = cv2.imread("./TeamCode/src/main/python/img.png")
    cropped_img = img.copy()
    hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask = mask0+mask1
    output_img = cropped_img.copy()
    output_img[np.where(mask==0)] = 0
    edged = cv2.Canny(output_img, 10, 200) 
    contours, hierarchy = cv2.findContours(edged,  
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
    cv2.drawContours(cropped_img, contours, -1, (0, 255, 0), 3) 
    return cropped_img

while success:
    success,image = vidcap.read()
    count += 1
    if image is None:
        continue
    newimg = imgrec(image)
    final_imgs.append(newimg)

video=cv2.VideoWriter('video.mp4',-1,60,(width,height))
for img in final_imgs:
    video.write(img)
video.release()
print("done")