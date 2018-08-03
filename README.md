# VEHICLE RISK-DETECTION BASED ON KINECT.

This repository contents: 
- Used database.
- source codes.
- dev scripts(Python | Matlab)


## Software requirements

- Linux Ubuntu 16.04 TLS
- Python 2.7.15,
*numpy, pandas scikit-lean.
- Matlab 2017 R1
- GCC 5.4.0
- G++ 5.4.0
- Cmake 4.1
- ROS kinetic 1.12.13

## ROS Installing.
For ROS install you must follow the next steps:
* http://wiki.ros.org/kinetic/Installation/Ubuntu

or follow the video:

* https://www.youtube.com/watch?v=36O6OGOJG1E

##  Installing Driver/Bridge Kinect for Linux.
You must configure KinectV2, for this you must configure the Driver

* https://github.com/OpenKinect/libfreenect2#debianubuntu-1404

then install the Bridge

* https://github.com/code-iai/iai_kinect2#install

## Repository Folders

```
/your_root        - Directorio
|-- ROS              / ROS environment
|--BaseData          / Database from Kinect V2
|--Dev_Python        / scripts based on: M. Weinmann, B. Jutzi, and C. Mallet (2014)
|--Dev_Matlab        / scripts in MATLAB
``` 

## Use Folders.
In the folder `ROS` , the contents of catkin_ws/src must be copied to your own catkin workspace.

the `BaseData` folder stores the data with which the classifiers were validated and the tests presented in the final document were performed.

The `dev_Python dev_Matlab` folders are the python classifiers and matlab view scripts.

## Use ROS

in a terminal you should run the ROS kernel with

`roscore`

on another tab the brigde is executed

`roslaunch kinect2_bridge kinect2_bridge.launch reg_metod:=cpu depth_method:=cpu publish_tf:=true fps_limit:=1`

to capture data is executed.

`rosrun my_pack k2_capture /{your_path}/{your_fileName}.txt`

## Related information:
* https://www.youtube.com/watch?v=2XR_hcvC-Hg



## Autores

* Harold F. Murcia -  (www.haroldmurcia.com)
* Wilford Mayorga - (ing.wilford@gmail.com)
* Duvier Jairo Lugo -  (jairolugo2002@hotmail.com)


## License 

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org>



