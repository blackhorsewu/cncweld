# cncweld

## <p align="center"> Chinese National Engineering Research Centre for Steel Construction (Hong Kong Branch) </p>
## <p align="center"> 国家钢结构工程技术研究中心 （香港分中心）</p>

### Introduction

This project tries to use a CNC Mechanism with 4 degrees of freedom (dof) to do autonomous Gas Metal Arc Welding (GMAW).

It was mainly designed and implemented by Victor W H Wu (邬伟雄).

This project uses a self built CNC Mechanism, driven by *stepper motors*. It uses a *laser scanner* - Keyence LJ V7200 to scan the welding groove. This will help users to locate the welding groove relative to the CNC machine without time consuming setup and alignment time.

The whole project is implemented in the Robot Operating System (ROS). As the laser scanner scans the welding groove, a point cloud showing the welding groove is shown in the RViz (ROS Visualizer). At the same time, *way-points* of the welding path is also shown.

When the scanning is completed, users can edit and even pick and choose whichever way-points are wanted or not wanted and where to begin and where to end.

After the way-points of the welding path is confirmed, the CNC Mechanism will start welding from either end as instructed by the user.

### Modules

#### 1. CNC Description

To control the CNC mechanism, it is necessary to build a model of it. ROS uses the Universal Robot Description Format (URDF) to describe robots. The CNC mechanism used in this project is a kind of robot, with 3 *prismatic* joints for the X, Y, and Z axes and a *rovolute* joint for the *A* axis. In CNC terms, the *A* axis is the rotating axis that rotates about the X axis. Therefore the *welding torch* can swing from side to side of a welding groove.








