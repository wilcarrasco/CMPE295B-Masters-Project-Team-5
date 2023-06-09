import cv2
import time
from time import sleep
import numpy as np
from elements.yolo import OBJ_DETECTION
from Transbot_Lib import Transbot

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

def drawCrossHair(frame: object, color: tuple=(0, 255, 0)):
    """
    Modifies supplied openCV frame and draws a crosshair in the center of the franme.
    This function makes an assumption of a 640 X 480 image

    Parameters
    ----------
    frame: object
        The image frame

    color: tuple, default = (50, 172, 50) Yellow-Green
        Color of the supplied text, defaults to yellow-green color
    """
    # Line thickness of 9 px
    thickness = 2

    start_point = (310, 240)
    end_point = (330, 240)
    frame = cv2.line(frame, start_point, end_point, color, thickness)

    start_point = (320, 230)
    end_point = (320, 250)
    frame = cv2.line(frame, start_point, end_point, color, thickness)

    return frame

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
        self.bot = Transbot()
        self.target_servox = 90
        self.target_servoy = 90
        self.x = 320
        self.y = 240
        self.target_aquired = 0
        self.resetServo()

    def resetServo(self):
        print("Servo Reset Start")
        self.target_servox = 90
        self.target_servoy = 90
        self.pwmServo(self.target_servox, self.target_servoy)
        time.sleep(1)
        self.bot.set_beep(0)
        time.sleep(1)
        self.bot.set_colorful_effect(0)
        time.sleep(1)
        self.floodLight(0)
        print("Servo Reset End")

    def pwmServo(self, x, y):
        self.bot.set_pwm_servo(1, x)
        self.bot.set_pwm_servo(2, y) # only modify this for HD camera
        return x, y

    def floodLight(self, intensity):
        self.bot.set_floodlight(intensity)

    def track(self, x, y):
        # we want to swing quickly but slow down as we converge tracking.
        adjustment = abs(x - self.x) // 100 + 1
        print("X servo adjustment: {}".format(adjustment))

        # Adjustment margin can be increased using +- 20 for now but helps with jitter
        # X Axis
        if x > (self.x + 20):
            self.target_servox -= adjustment
            if self.target_servox < 0:
                self.target_servox = 0
        elif x < (self.x - 20):
            self.target_servox += adjustment
            if self.target_servox > 180:
                self.target_servox = 180

        adjustment = abs(y - self.y) // 100 + 1
        print("Y servo adjustment: {}".format(adjustment))

        # Adjustment margin can be increased using +- 20 for now but helps with jitter
        # X Axis
        if y > (self.y + 20):
            self.target_servoy += adjustment
            if self.target_servoy < 0:
                self.target_servoy = 0
        elif y < (self.y - 20):
            self.target_servoy -= adjustment
            if self.target_servoy > 180:
                self.target_servoy = 180

        # Update tracking
        self.pwmServo(self.target_servox, self.target_servoy)

    def process(self, frame):
        objs = self.object_detector.detect(frame)
        tracking = False

        # plotting if focus object is found (person)
        if len(objs) > 0:
            tracking = True
            obj = objs[0]
            label = obj['label']
            score = obj['score']
            [(xmin,ymin),(xmax,ymax)] = obj['bbox']
            color = self.object_colors[self.object_classes.index(label)]
            frame = cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2) 
            frame = cv2.putText(frame, f'{label} ({str(score)})', (xmin,ymin), cv2.FONT_HERSHEY_SIMPLEX , 0.75, color, 1, cv2.LINE_AA)

            # Get the center position of detection frame
            x_pos = (xmin+xmax) // 2
            y_pos = (((ymin+ymax) // 2) + ymin) // 2

        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        drawText(frame, "FPS: " + str(int(fps)), (20, 40))
        frame = drawCrossHair(frame)
        cv2.imshow("TransBot Camera", frame)
        cv2.imwrite("test.jpg", frame)

        # need to update this tracking boolean to be a counter
        if tracking:
            self.target_aquired += 1
            print("xpos: {} | ypos: {}".format(x_pos, y_pos))
            self.track(x_pos, y_pos)
            if self.target_aquired > 60:
                self.target_aquired = 60
        else:
            self.target_aquired -= 1
            if self.target_aquired < 0:
                self.target_aquired = 0

        # If object is detected after 30 frames turn in buzzer, spotlight, led bar
        # If tracking is lost after 30 frames, turn off detection sounds
        if self.target_aquired > 30:
            self.bot.set_beep(1)
            self.bot.set_colorful_effect(2, 255)
            self.floodLight(100)
        elif self.target_aquired < 30:
            self.bot.set_beep(0)
            self.bot.set_colorful_effect(0)
            self.floodLight(0)

    # Dumb destructor
    def __del__(self):
        del self.bot
        print("Destroying Yolo Tracker")

if __name__ == '__main__':
    YoloTracker = yoloTrack()
    cap = cv2.VideoCapture(0) # Using the HD camera can be changed for the Depth camera as required
    if cap.isOpened():
        window_handle = cv2.namedWindow("TransBot Camera", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("TransBot Camera", 0) >= 0:
            ret, frame = cap.read()
            keyCode = cv2.waitKey(10) & 0xFF
            if ret:
                timer = cv2.getTickCount()

                # detection process
                YoloTracker.process(frame)

            # Change to escape key press | ordinal keys caused a segfault due to bug
            if keyCode == 27:
                break
        print("Stop YOLO TRACKER")
        YoloTracker.resetServo()
        sleep(1)
        cap.release()
        cv2.destroyAllWindows()
        sleep(1)
        del YoloTracker
        print("TERMINATE Program")
    else:
        print("Unable to open camera")
