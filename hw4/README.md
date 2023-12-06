# Homework 4
For Homework 4 we define the ros package `hw4_msgs` to contain our `CameraLocalization` custom message.

## CameraLocalization Message
The `CameraLocalization` message header is as follows:
```
float64 alignment_error
float64 distance_separation
```

`alignment_error` is in angles from center of camera in screen space along the x-axis, centered at 0. Therefore, negative values correspond to "left."

`distance_separation` is in meters from marker center.

## Using CameraLocalization in your node
An example setup for a new node that uses `CameraLocalization` (perhaps q2 or q3) is as follows:

First, clone the `tbd` repository into `catkin_ws/src/`

Then, `cd` into `catkin_ws/src/tbd/hw4` and make a new package:
```bash
catkin_create_pkg q2
```

Since setting up a node with package dependencies is non-trivial, to make things easy we will just copy both `package.xml` and `CMakeLists.txt` from the `q1` package into the `q2` package into the same "spots" under `catkin_ws/src/tbd/hw4/q2/`.

Then, go into both `package.xml` and `CMakeLists.txt` that you copied into `q2` and find and replace all instances of `q1` to `q2`. Note, these configuration files assume you have placed your `q2.py` for example under `catkin_ws/src/tbd/hw4/q2/scripts/`. The relevant spot in `CMakeLists.txt` for this configuration is where it reads:
```
catkin_install_python(PROGRAMS
    scripts/q1.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Therefore, you must follow this guideline or change this part of `CMakeLists.txt` to reflect your change in name or path.

Once complete, we will then clean `/build` and `/devel` with:
```bash
catkin_make clean
```

Then we will rebuild all of our packages, which will generate the `python` module for our `CameraLocalization` message (so we can do `from hw4_msgs.msg import CameraLocalization` in our node) with:

```bash
catkin_make
```

The `CameraLocalization` message should now have been built, and you can check with:
```bash
rosmsg list
```
and hopefully find it under `hw4_msgs/CameraLocalization`.

Lastly, you may now attempt to execute your ros package that now depends on our custom message.
First, source your environment:
```bash
source ./devel/setup.bash
```

Then you should be able to execute your new package with:
```bash
rosrun q2 q2.py
```

For a sanity check, you may also try running `q1`:
```bash
rosrun q1 q1.py
```
*Note* - `q1` uses a named window from `opencv` which will not work when running on a server version of a linux distro, so modifications will be in order. Look to the latest version of `q2c` from our demo raspberry pi.

Also, make sure to perform the necessary `sudo chmod 777 /dev/video{X}`'s to ensure the camera device has necessary permissions.s