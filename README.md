# comp3431_17s2_hsr_pickup_object_and_carry_me
### User Manual on how to launch object recognition code

**Package Name** : hsrb\_darknet\_tutorials

**Files** :

1. **launch/default\_model\_demo.launch**

1.
  1. **Description** : This is a file that should be on the HSR Robot and it is in charge of running YOLO. It subscribes to the colour image on HSR Robot and runs YOLO on the received image. It then publishes the converted image with the detected object  as well as an array of the detected object centre and the boundary height and width.
  2. **Arguments** : None
  3. **Note** : The first thing you need to do before doing anything is to run yolo on the GPU if you don&#39;t you get an error. If you get the error the only thing you can do is to restart the HSR and try again. You also need be logged into the level 3 research network whose password is crispyink133.
  4. **How to run** :

                        $ ssh -x ubuntu@unsw-hsrb-tk1.local

                        Passwod :  dt3gpu

                        $ cd workspace/indigo\_workspace/

                        $ source devel/setup.bash

                        $ export ROS\_MASTER\_URI= http://unsw-hsrb.local:11311/

                        $ export ROS\_HOSTNAME=unsw-hsrb-tk1.local

                        $ roslaunch hsrb\_darknet\_tutorials default\_model\_demo.launch

**Package Name** : myvis

**Files** :

1. **src/ToPickFinal.cpp**

1.
  1. **Description** : ToPickFinal.cpp is a publisher that read in the command line and prints that value as the object to pick. It will only take in the first value as the string to publish  so if the object&#39;s name contain multiple word you will need to put it in single quotation.
  2. **Arguments** : &lt;Object\_to\_pick&gt;
  3. **How to run** :

                      $ cd workspace

                      $ hsrb\_mode

                      $ source devel/setup.bash

                      $ rosrun myvis ToPickFinal.py &#39;bottle&#39;

1. **src/visionfinal.cpp**

1.
  1. **Description** : This code subscribes to the array of centres and the boundary height and width from YOLO. It uses this data to calculate the xyz global and local positions and to publishes Objects.msg for both the local and global frame.
  2. **Arguments** : None
  3. **How to run** :

                      $ cd workspace

                      $ hsrb\_mode

                      $ source devel/setup.bash

                      $ rosrun myvis visionfinal.py

1. **msg/tables.msg**

1.
  1. **Description** : The data being transferred over ROS which contains an array of table

1. **msg/table.msg**

1.
  1. **Description** : The data being transferred over ROS which contains the centre of the table in xyz position and the min and max xyz for the table.

1. **msg/Object.msg**

1.
  1. **Description** : The data being transferred over ROS which contains the centre of the Object in xyz position and the camera x and y position. The camera x and y position are also used to store height and width when ObjectPCL is publishing an object.

1. **msg/Objects.msg**

1.
  1. **Description** : The data being transferred over ROS which contains an array of Object

**Package Name** : opencvcode

**Files** :

1. **src/table.cpp**

1.
  1. **Description** :  This code subscribes to the point cloud and finds all the tables. It then publishes the point cloud it see one by one. It should also publish an array of all the tables but that has been implemented yet. This file is on the HSR Robot.
  2. **Arguments** : None
  3. **How to run** :

                      $ ssh -X hsr-user@unsw-hsrb.local

                      Password : dt3lap

                      $ cd workspace/indigo\_workspace/

                      $ source devel/setup.bash

                      $ rosrun opencvcode table

1. **src/pointcloud\_sub.cpp**

1.
  1. **Description** : This code subscribes to the point cloud and as well as the visionfinal array of Object. The first thing the code does is to match the point cloud cluster with the objects. The code also subscribes to the object to pick and if the name matches the objects to pick it publishes the point cloud of that cluster as well as the object to pick with xyz position and height and width.
  2. **Arguments** : None
  3. **How to run** :

                      $ ssh -X hsr-user@unsw-hsrb.local

                      Password : dt3lap

                      $ cd workspace/indigo\_workspace/

                      $ source devel/setup.bash

                      $ rosrun opencvcode ObjectPCL

**Prior Installation:**

PCL: [http://wiki.ros.org/pcl](http://wiki.ros.org/pcl)

Opencv\_vison: [http://wiki.ros.org/vision\_opencv](http://wiki.ros.org/vision_opencv)

**RUN order:**

Default\_model\_demo.launch

ToPickFinal.cpp

visionfinal.cpp

Pointcloud\_sub.cpp

table.cpp
