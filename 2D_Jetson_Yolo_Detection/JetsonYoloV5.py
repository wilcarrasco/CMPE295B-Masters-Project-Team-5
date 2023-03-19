import cv2
import numpy as np
from elements.yolo import OBJ_DETECTION

def drawText(frame, txt, location, color = (50, 172, 50)):
    cv2.putText(frame, txt, location, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

Object_classes = open('./weights/coco.names').read().strip().split('\n')
Object_colors = list(np.random.rand(80,3)*255)

print("Initialize Model: Start Loading in YOLOv5 Weights")
Object_detector = OBJ_DETECTION('weights/yolov5s.pt', Object_classes)
print("Initialize Model: Complete")

# Select Correct Camera
cap = cv2.VideoCapture(1)
if cap.isOpened():
    window_handle = cv2.namedWindow("JetBot Camera", cv2.WINDOW_AUTOSIZE)
    # Window
    while cv2.getWindowProperty("JetBot Camera", 0) >= 0:
        ret, frame = cap.read()
        if ret:
            timer = cv2.getTickCount()
            # detection process
            objs = Object_detector.detect(frame)

            # plotting
            for obj in objs:
                # print(obj)
                label = obj['label']
                score = obj['score']
                [(xmin,ymin),(xmax,ymax)] = obj['bbox']
                color = Object_colors[Object_classes.index(label)]
                frame = cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2) 
                frame = cv2.putText(frame, f'{label} ({str(score)})', (xmin,ymin), cv2.FONT_HERSHEY_SIMPLEX , 0.75, color, 1, cv2.LINE_AA)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        drawText(frame, "FPS: " + str(int(fps)), (80, 100))
        cv2.imshow("JetBot Camera", frame)
        cv2.imwrite("test.jpg", frame)
        keyCode = cv2.waitKey(30)
        if keyCode == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
else:
    print("Unable to open camera")
