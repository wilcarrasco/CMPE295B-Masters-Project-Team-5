#### Hardaware
Jetson Nano Developer Kit 4GB

### Camera
Yahboom HD Camera

Use the following command to check if the camera is recognized correctly.
```
$ ls /dev/video0
```

### OpenCV 4.5.5 Installation to run on the GPU/CUDA
Install OpenCV 4.5.5 - THIS WILL TAKE A LONG TIME / plan starting with a full battery or unplug the 40 pin connector and power the nano via 2 amp usb charger [**here**](https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html)

You can increase swap using the opencv 4.5 page or this [**url**](
https://jetsonhacks.com/2019/11/28/jetson-nano-even-more-swap)

### YOLOv3 Weights
You must download the YOLOv3 weights file from [**here**](https://github.com/patrick013/Object-Detection---Yolov3/blob/master/model/yolov3.weights) and place it in the ```Jetson_Detector_Tracker``` folder.


## Inference
Run ```DetectionTracker.py``` to detect objects with the camera.
```
$ python3 DetectionTracker.py
```
[Video File Recordings](https://drive.google.com/drive/folders/1WhzUWYhf2UZgFSYVY8M2zptHVlV6JOr9?usp=share_link)