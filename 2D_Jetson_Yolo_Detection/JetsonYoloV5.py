import cv2
import numpy as np
from elements.yolo import OBJ_DETECTION

def drawText(frame: object, txt: str, location: tuple, color: tuple=(50, 172, 50)):
    """
    Modifies supplied openCV frame and draws text.

    Parameters
    ----------
    frame: object
        The image frame

    txt: string
        Text to write on the frame

    location: tuple
        Location of text in relation to the frame

    color: tuple, default = (50, 172, 50) Yellow-Green
        Color of the supplied text, defaults to yellow-green color
    """
    cv2.putText(frame, txt, location, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

class yoloTrack(object):
    """
    2D YOLOv5s Focus object detector and servo tracker class that can detect and track a single specified CoCo object
      in a series of frames.

    Parameters
    ----------
    focus_object: str, default = 'person'
        Sets the object type to focus on detecting, drawing, and servo tracking
    """
    def __init__(self, focus_object='person'):
        self.object_classes = open('./weights/coco.names').read().strip().split('\n')
        self.object_colors = list(np.random.rand(80,3)*255)
        self.focus_object = focus_object
        print("Initialize Model: Start Loading in YOLOv5 Weights")
        self.object_detector = OBJ_DETECTION('weights/yolov5s.pt', self.object_classes , self.focus_object)
        print("Initialize Model: Complete")

    def process(self, frame):
        objs = self.object_detector.detect(frame)

        # plotting if focus object is found
        if len(objs) > 0:
            obj = objs[0]
            label = obj['label']
            score = obj['score']
            [(xmin,ymin),(xmax,ymax)] = obj['bbox']
            color = self.object_colors[self.object_classes.index(label)]
            frame = cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2) 
            frame = cv2.putText(frame, f'{label} ({str(score)})', (xmin,ymin), cv2.FONT_HERSHEY_SIMPLEX , 0.75, color, 1, cv2.LINE_AA)

        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        drawText(frame, "FPS: " + str(int(fps)), (20, 40))
        cv2.imshow("TransBot Camera", frame)
        cv2.imwrite("test.jpg", frame)

if __name__ == '__main__':
    YoloTracker = yoloTrack()
    cap = cv2.VideoCapture(0) # Using the HD camera can be changed for the Depth camera
    if cap.isOpened():
        window_handle = cv2.namedWindow("TransBot Camera", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("TransBot Camera", 0) >= 0:
            ret, frame = cap.read()
            if ret:
                timer = cv2.getTickCount()

                # detection process
                YoloTracker.process(frame)

            keyCode = cv2.waitKey(3)
            if keyCode == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")
