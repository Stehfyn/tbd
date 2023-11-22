Source your env

```bash
source ./devel/setup.bash
sudo chmod 777 -R src/
```

Ensure lidar is connected as a /dev/ttyUSBX
```bash
lsusb
ls /dev
sudo chmod 777 -R /dev 
```

Roscore
```bash
roscore
```

Publisher
```bash
rosrun q1a q1a_publish.py
Ctrl+C to exit
```

Subscriber
```bash
rosrun q1a q1a_subscribe.py
```

Generate the plotted data .png
```bash
python3 src/tbd/project2/q1a/scripts/q1a_plot.py
```