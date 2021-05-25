
# Temperature mapping with TurtleBots and ROS

## Introduction
Data is very important. Be it weather data, inventory data, financial data, etc. As we rely on data immensely, acquiring data is equally important. This report describes a project that aims to map the temperature of an environment using ROS (Robotic Operating System) and Turtlebots. ROS is a rapidly developing framework with a lot of community-driven work for Robotics and Turtlebots are cost-effective and developer friendly for beginners.

### Goals:
1. Map the environment
2. Collect temperature data at discretized points in the environment (ideally autonomously, but this report describes manual collection of temperature data)
3. Visualize the temperature distribution 

## Software and Hardware requirements:
The following are the tools or specifications of software and hardware used for this project:

• Turtlebot 2, with its laptop (referred to as ASUS laptop) - ASUS laptop with Ubuntu 14.04, and ROS Indigo 

• Hokuyo LIDAR sensor – URG-04LX --> Range : 60mm - 4095mm; Operating voltage - 5V

• KY-013 Arduino temperature sensor module -->  Range : -55°C to 125°C; Operating voltage - 5V

• Arduino Uno

• Laptop (Workstation) - Ubuntu 14.04 or higher, either through virtual machine or boot-up.

• WiFi router

### Basic installion guide:
- For this project, on the workstation Ubuntu 20.04 was installed. Follow the [link](https://ubuntu.com/download/desktop) for installing it.

- Ubuntu is run as a virtual machine in the workstation. Follow the [link](https://my.vmware.com/web/vmware/downloads/#all_products) to install the VMware workstation player. 

- ROS Noetic Ninjemys was installed on the workstation. Follow the [link](http://wiki.ros.org/noetic/Installation/Ubuntu) to install the same. This [video](https://www.youtube.com/watch?v=9U6GDonGFHw) may help you do the same.

- Few packages need to be installed on the ASUS laptop for operating TurtleBots. Follow the [link](http://wiki.ros.org/turtlebot/Tutorials/indigo/Turtlebot%20Installation) to install them. 

- Follow the [tutorial](http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration) to configure the network between the ASUS laptop and the workspace. To test the connection, open the terminal on the workstation and run `ssh -X [username of ASUS laptop]@[IP of ASUS laptop on the network]`, enter the password and the connection is secure. `-X` servers are used for enabling visual data stream which is required for RViz. For example, `ssh -X turtlebot@192.168.0.3`

- Download the repository on the ASUS laptop, extract the `TurBot_ws` and navigate into the workspace `TurBot_ws` through the terminal and build the catkin workspace by running `catkin_make`.

- Ensure that the above mentioned catkin workspace is called in the `.bashrc` file of the ASUS laptop. To check the file run `gedit .bashrc` on the terminal. To include the workspace, follow the instructions in the [link](https://www.youtube.com/watch?v=xgLETnSMMYA&list=PLSzYQGCXRW1H8R2Bok_K8wcsE12_49alQ&index=6)
 
-Check out the [link](https://arduinomodules.info/ky-013-analog-temperature-sensor-module/) to connect the temperature sensor module to the Arduino Uno. The Arduino file `temperaturePublishToROS.ino` creates a node that continuously publishes the temperature data every second. 
- Ensure that both the TurtleBot and the ASUS laptop are well charged.

- Connect the LIDAR sensor to the TurtleBot. Connect both the TurtleBot and the Arduino Uno to the ASUS laptop.

Now both the systems are ready to manually map the temperature of the environment.

## Workflow
*Note: All the steps regarding terminal tabs on the workstation assume that a ssh connection has been made with the TurtleBot*
### To map the environment:
- On the terminal of the ASUS laptop run `roslaunch turtlebot_bringup minimal.launch`. Place the ASUS laptop on the TurtleBot and leave it in the environment.

- On the terminal of the workstation run `roslaunch turtlebot_navigation hokuyo_gmapping_demo.launch`.

- On another tab run `roslaunch turtlebot_rviz_launchers view_navigation.launch`. This should initiate RViz, and it will show the visual data recorded by the LIDAR sensor.

- On another tab run `
roslaunch turtlebot_teleop keyboard_teleop.launch`. With RViz open, and the fore-mentioned tab open, navigate the TurtleBot around the environment, ensuring that the map has no gaps.


- Once the environment is clearly mapped, close the `keyboard_teleop` node by using the keystroke `Ctrl+c`

- Save the map by running `
rosrun map_server map_saver -f /home/turtle/TurBot_ws/src/turtlebot_apps/turtlebot_navigation/maps/Map` on the terminal.

### To map the temperature of the environment
- Upload the Arduino file `temperaturePublishToROS.ino` to the Uno board. Note the port number to which the Arudino is connected to. For example, `/dev/ttyACM1`

- Navigate to `launch` in `turtlebot_navigation` in `TurBot_ws`. Open the file `amcl_hoku.launch` and modify the `default` field with `arg_name` as `map_file` to include the absolute address of the map file that was just created

- On a terminal tab on the ASUS laptop run `roslaunch turtlebot_bringup minimal.launch`. 

- On another terminal tab on the ASUS laptop run `rosrun rosserial_python serial_node.py /dev/ttyACM1`. Place the ASUS laptop on the TurtleBot and leave it in the environment.

- On a terminal tab on the workstation run `roslaunch turtlebot_navigation amcl_hoku.launch`. This will initialize localization algorithms.

- On another terminal tab on the workstation run `roslaunch turtlebot_rviz_launchers view_navigation.launch` to visualize the TurtleBot in the map. RViz should open up with the TurtleBot localized at a random point on the map, with green arrows under it. 

- On another terminal tab on the workstation run `rosservice call /global_localisation "{}"`. This service initializes localisation of the robot in the map by assigning equal probabilities among all the locations in the map which is seen in RViz as green arrows being dispersed all along. 

- To help the TurtleBot loacalize itself, it should be moved around in the map for a brief period of time for the localization algorithms to converge to a solution of its position. To do this, run `roslaunch turtlebot_teleop keyboard_teleop.launch` and navigate the TurtleBot in the environment until it is localized, i.e. the arrows are all under the TurtleBot in the RViz. The position of the TurtleBot in the environment should be mimicked by the TurtleBot in the map on RViz.

- Once localization is done, we are ready to map the temperature of the environment. To do this, on another terminal tab of the workstation run `rosrun beginner_tutorials listen.py`

- Go back to the terminal tab where the `keyboard_teleop` node was launched and use it navigate the TurtleBot around the environment. Having navigated the TurtleBot all around the environment, close all the terminal tabs and navigate to `scripts` in `beginner_tutorials` in the `TurBot_ws` workspace to find `TempData.csv`. This file has temperature data for respective points on the map. 

### To visualize temperature data
- Copy the map - `Map.pgm`, `Map.yaml` and the `TempData.csv` file into the `Temp Data Visual` folder and then run the `TempVis.py` to generate `TempVisual.png`. This is an image file with the temperature data superimposed upon it .

## Notable challenges
• Mapping with LIDAR:
The initial steps of bringing up the turtlebot and running the gmapping module to map an area with the Hokuyo LIDAR has been exhaustive and took long for us to figure it out. Although this is relatively an easier task, we struggled due to the fact that we were beginners in ROS and just started exploring its framework.
The problem was that the right workspace was not being accessed when running various files. This was solved by including the right workspace in the `.bashrc` file.
This problem helped us better understand the fundamentals
and, in the end, we figured it out.

• Decoding the generated map & way point generation:
Understanding the map and the associated .yaml file and then writing a program to take this map and generate a sequence of co-ordinates of the way points for a trajectory evenly covering the whole map. Additionally, a trajectory map of sorts is created which inflates the borders to avoid collision .This has been implemented in the python code `imageProcessing.ipynb`.

• Navigation Goals:
This is sending goals to the navigation stack of the TurtleBot for it to reach a specific point in a map when placed in the same area represented in the map. This has been achieved by using RViz, but to implement it using code we are still working. One recent cause we found can be linked to the fact that it is difficult for the robot to localize itself in a symmetric area and hence we are working to fix this. This step is important for autonomous temperature mapping.

• Temperature Data Visualization:
The challenge of an aesthetic visualization of the temperature data vs space. This is implemented in `TempVis.py` script. The temperature points are color graded in the output image.

## Current status
Manual mapping of the environment is successfully done. Also the temperature of the map is manually mapped. The temperature data collected can also be visualized on an image using the python code `TempVis.py`. In short, manual mapping and visualization are successfully implemented.

## Further work

This involves autonomously navigating the known map and collecting temperature values by a following a series of way-points generated for that map. This couldn't be done as we were unable to implement the sending of navigation goals.
 
The file `blah.py` in `scripts` in `beginner_tutorials` is an attempt to enable the TurtleBot move to a given point in the map autonomously. Given a list of points in the file `blah.py`, the TurtleBot should be able to navigate through them respecting the order. A ROS service file should be authored which when called must return the `amcl_pose` and the temperature data at a given point only. Once the above has been successfully implemented, the integration of  `imageProcessing.ipynb` & `blah.py` must be done. The code in `listen.py` can be used to implement the previously mentioned service which when called in `blah.py`, the TurtleBot will be able to get its `amcl_pose` and temperature value for a point and the whole data be saved to `TempData.csv`.

## References
- [Tutorial to download the workspace for TurtleBots](https://www.ncnynl.com/archives/201611/1097.html)

- [ROS wiki](www.wiki.ros.org)

- [Virtual machines](https://www.youtube.com/watch?v=ehtUb55Rmmg&list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC&index=1)

- [ROS basics](https://www.youtube.com/watch?v=xgLETnSMMYA&list=PLSzYQGCXRW1H8R2Bok_K8wcsE12_49alQ&index=6)

- [Arduino on Linux](https://www.arduino.cc/en/guide/linux)
- [Arduino ROS setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

- [Arduino ROS 'Hello World'](http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World)

- [Temperature moduel configuration](https://arduinomodules.info/ky-013-analog-temperature-sensor-module/)

## Result

The temperature sensor is configured to collect readings and publish them continuously in a topic through a node. We have manually navigated the robot in the known map with the program  `listen.py` that collects these temperature values and the pose (amcl) of the robot in the map at regular intervals of time and save them in a csv file. The following image is the map image of the environment.

![Environment map](https://github.com/gsShourie/NinjaTurtle/blob/master/docs/Map.png)

The following image shows a snippet of the temperature data in the TempData.csv

![TempData.csv snippet](https://github.com/gsShourie/NinjaTurtle/blob/master/docs/csvfile.png)

This csv file with temperature & coordinate data along with the map file is passed to the python script to generate the final image of the map with a color-based representation of temperature at the specific points. The below image doesn’t have much variation due to the fact that this whole measurement was performed in a closed room without any considerable temperature changes.
The following image shows the temperature data integrated with the map.
![Temperature data with map](https://github.com/gsShourie/NinjaTurtle/blob/master/docs/TempVisual%20with%20legend.png)


## Authors
[Prithvi Bharadwaj M.](https://sites.google.com/view/prithvi-bharadwaj-m/home)

[Shourie G. Srinivas](www.shouriegs.com)
