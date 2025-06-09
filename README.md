# Arducam_TOF_ROS1

This repo contains installation instruction for Arducam TOF B0410C on a Jetson Nano 

#SYSTEM INFO

- Ubuntu 18.04
- Jetson Nano
- ROS 1 Melodic
- Python2
- 
#INSTALLATION STEPS

Step 1: Clone this repo
```bash
  git clone https://github.com/ArduCAM/Arducam_tof_camera.git
  cd Arducam_tof_camera
```
Step 2: Install dependencies

```bash
  ./Install_dependencies_jetson.sh
```
Step 3: Error troubleshooting

If you have this error:

Failed to install ArducamDepthCamera.
python -m pip install ArducamDepthCamera opencv-python "numpy<2.0.0"

Do:

```bash
  python -m pip install ArducamDepthCamera "numpy<2.0.0"
``` 
We don't include opencv-python as it is a fake error noti.
After this step, reboot as the prompt goes.

Step 4: Complile

```bash
  ./compile.sh
```



