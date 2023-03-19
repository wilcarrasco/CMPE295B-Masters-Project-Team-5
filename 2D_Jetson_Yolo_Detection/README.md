#### Hardaware
Jetson Nano Developer Kit 4GB

#### Camera
Logitech HD Pro Webcam C920

Use the following command to check if the camera is recognized correctly.
```
$ ls /dev/video0
```

##### PyTorch & torchvision
Yolov5 network model is implemented in the Pytorch framework.
PyTorch is an open source machine learning library based on the Torch library, used for applications such as computer vision and natural language processing.
Heres a complete guide to [**install PyTorch & torchvision**](https://forums.developer.nvidia.com/t/pytorch-for-jetson-version-1-9-0-now-available/72048) for Python on Jetson Development Kits

## Inference
Run ```JetsonYoloV5.py``` to detect objects with the camera.
```
$ python3 JetsonYoloV5.py
```
[Video File Recordings](https://drive.google.com/drive/folders/18rL822lbXBCEumcBSIpDz1um5rX6ugdv?usp=sharing)
