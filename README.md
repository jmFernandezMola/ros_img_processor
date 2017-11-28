## ros_img_processor
Afegida la funcionalitat de detecció de cercles. Internament calcula el raig que apunta al centre del cercle. Per tal de poder fer servir el raig, s'ha implementat un *publisher* que publica la *pose* amb origen (0,0,0) i direcció la del vector. 

Per poder implementar aquesta tasca, s'ha llegit el topic */usb_cam/camera_info* de la càmera calibrada, concretament la matriu K. El propi codi la inverteix per poder fer l'operació d=K^{-1}·u


## How to run the code
In a terminal window, type:
```sh
$ roslaunch ros_img_processor ros_img_processor.launch
```

## Tip
Check your webcam encodings (yuyv,mjpeg,...) and set them accordingly at file launch/usb_camera.launch
