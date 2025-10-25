import cv2
from ultralytics import YOLO

modelo = YOLO('y11_test1.pt')

img = cv2.imread('gazebo.png')
resultado = modelo.predict(img, verbose=False)

for obj in resultado:
    obj.show()
