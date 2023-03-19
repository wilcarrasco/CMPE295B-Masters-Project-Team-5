#!/usr/bin/env python3
# -*-coding: utf-8 -*-

import cv2
import numpy as np
import time


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
    cv2.putText(frame, txt, location, cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 3)

def getOutputLayers(net: object):
    """
    Returns the layers in the supplied Deep Neural Netowork

    Parameters
    ----------
    net: object
        Initialized neural network


    Returns
    -------
        ln: array
            Returns an array of strings that describes the model's connected output layers
    """
    ln = net.getLayerNames()
    ln = [ln[i - 1] for i in net.getUnconnectedOutLayers()] 
    return ln

class DetectorTracker(object):
    """
    2D Human object detector and tracker class that can detect and track a single peron in a series of frames

    Parameters
    ----------
    obj_score: float, default = 0.5
        Object objectiveness score, minimum probability value

    conf_score: float, default = 0.5
        Object confidence score, minimun

    nms_score: float, default = 0.5
    """
    def __init__(self, obj_score: float=0.5, conf_score: float=0.5, nms_score: float=0.5):
        self.obj_score = obj_score
        self.conf_score = conf_score
        self.nms_score = nms_score
        print("Initialize Model: Start Loading in YOLOv3 Weights")
        self.model = cv2.dnn.readNetFromDarknet('yolov3.cfg','yolov3.weights')
        self.model.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.model.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.tracker = cv2.TrackerKCF_create()
        print("Initialize Model: Complete")
        self.ln = getOutputLayers(self.model)
        self.object_classes = open('coco.names').read().strip().split('\n')
        self.object_colors = list(np.random.rand(80,3)*255)

        self.human_detect = False
        self.detect_count = 0
        self.tflag = False
        self.fail = 0
        self.retract = 0

    def trackObjects(self, frame, model):
        blob = cv2.dnn.blobFromImage(frame, 1/255, (192, 192), swapRB=True, crop=False)
        model.setInput(blob)
        output = model.forward(self.ln)
        tbox, cframe = self.drawDetection(frame, output)
        return tbox, cframe

    def drawDetection(self, frame, outputs):
        img_height = frame.shape[0]
        img_width = frame.shape[1]

        classids = []
        confs = []
        boxes = []

        for output in outputs:
            for detected in output:
                if detected[4] > self.obj_score:
                    scores = detected[5:]
                    classid = np.argmax(scores)
                    conf = scores[classid]
                    if conf > self.conf_score and self.object_classes[classid] == 'person':
                        center_x = int(detected[0] * img_width)
                        center_y = int(detected[1] * img_height)
                        w = int(detected[2] * img_width)
                        h = int(detected[3] * img_height)
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        classids.append(classid)
                        confs.append(float(conf))
                        boxes.append([x, y, w, h])

        indices = cv2.dnn.NMSBoxes(boxes, confs, self.conf_score, self.nms_score)
        
        if len(indices) > 0:
            i = indices[0]
            boxes = boxes[i]
            (x, y) = (boxes[0], boxes[1])
            (w, h) = (boxes[2], boxes[3])
            color = (255, 0, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(self.object_classes[classids[i]], confs[i])
            drawText(frame, text, (x, y - 5), color)
        return boxes, frame

    def process(self, frame):

        if self.fail >= 10 or self.retract >= 20:
            print("human_detect: {} | detect_count: {} | tflag: {} | fail: {} | retract: {}".format(self.human_detect,
                self.detect_count, self.tflag, self.fail, self.retract))
            self.human_detect = False
            self.detect_count = 0
            self.tflag = False
            self.fail = 0
            self.retract = 0

        if self.human_detect == False and self.detect_count < 60:
            tbox, frame = self.trackObjects(frame, self.model)

            if len(tbox) > 0:
                self.detect_count += 1
                drawText(frame, "Human Detected", (20, 110))

                if (self.detect_count >= 30):
                    self.human_detect = True
                    self.tracker = cv2.TrackerKCF_create()
                    self.tracker.init(frame, (tbox[0],tbox[1],tbox[2],tbox[3]))

        elif self.human_detect == True:
            rett, bbox = self.tracker.update(frame)

            if rett == True:
                self.tflag = True
                pt1 = (int(bbox[0]), int(bbox[1]))
                pt2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, pt1, pt2, (0,255,0), 3)
                cv2.putText(frame, "Person", (pt1[0]-5,pt1[1]-5),cv2.FONT_HERSHEY_COMPLEX,0.65,(255,255,0),2)
                cv2.putText(frame,"KCF Track", (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),2)
                self.retract += 1
            else:
                self.tflag = False
                tbox, frame = self.trackObjects(frame, self.model)

                if len(tbox)==0:
                    cv2.putText(frame, "Tracking failure detected", (20,80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
                    cv2.putText(frame,"KCF Detect", (20,20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),2)
                    self.fail += 1
                else:
                    self.tracker=cv2.TrackerKCF_create()
                    self.tracker.init(frame, (tbox[0],tbox[1],tbox[2],tbox[3]))
                    self.tflag = True
        return frame


if __name__ == '__main__':
    detectTrack = DetectorTracker()
    cap = cv2.VideoCapture(0)
    cv2_edition = cv2.__version__

    if cv2_edition[0] == '3':
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'XVID'))
    else:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : {}".format(cap.get(cv2.CAP_PROP_FPS)))
    print("OpenCV Version: {}".format(cv2.__version__))

    while cap.isOpened():
        start = time.time()
        ret, frame = cap.read()
        action = cv2.waitKey(1) & 0xFF

        #business logic here please.
        frame = detectTrack.process(frame)

        end = time.time()
        fps = 1 / (end - start)

        text = "FPS : {}".format(str(int(fps)))
        drawText(frame, text, (20, 50), (100, 200, 200))

        cv2.imshow("Transbot Camera", frame)

        if action == ord('q') or action == 113:
            break
    cap.release()
    cv2.destroyAllWindows()

