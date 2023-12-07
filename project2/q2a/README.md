First, start the pi only connected to the pioneer.

Wait until green light on pi, then connect the controller
Once the controller is connected, connect the camera.


```bash
source ./devel/setup.bash
```

```bash
sudo chmod 777 -R /dev
```

```bash
roslaunch p2os_launch p2os_teleop_joy.launch
```

```bash
rosrun q2a q2a.py
```

Push the enable motors button on robot


Hold bumpers, rstick for in place rotation
release all buttons, then press A for picture, wait atleast a full couple seconds before taking another
Repeat this sequence 3 additional times

Images will populate the current_directory