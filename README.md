# Robot Vision
Object Segmentation and Clustering in 3D Point Cloud using PCL
(ROS + Gazebo + RViz + PCL)

### Setup:
Clone the repository using:

    git clone https://github.com/anubhav1772/robot_vision.git

Run catkin_make in your ROS source directory

    $ cd ~/catkin_ws
    $ catkin_make

Start the simulation using:

    $ roslaunch robot_vision vision.launch
    

### Segmented Objects in RViz
<table>
  <tr>
    <td><img src="https://drive.google.com/uc?export=view&id=17eVxO6y37G5ZEx69LwgOBe1REDB0igIF" width=350 height=150></td>
    <td><img src="https://drive.google.com/uc?export=view&id=13rdrFMm3Oydf0mmfpXHnc65KUgBe1FLu" width=350 height=150></td>
  </tr>
 </table>

### Requirements
* [pcl v1.13.1](https://github.com/PointCloudLibrary/pcl/releases) - [Installation](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html)
* ROS Noetic (Ubuntu 20.04)
* Gazebo v11.11.0






