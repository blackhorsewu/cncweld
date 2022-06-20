## <p align="center"> Chinese National Engineering Research Centre for Steel Construction (Hong Kong Branch) </p>
## <p align="center"> 国家钢结构工程技术研究中心 （香港分中心）</p>

# CNCWELD

## Introduction

This project tries to use a CNC Mechanism with 4 degrees of freedom (dof) to do autonomous Gas Metal Arc Welding (GMAW).

It was mainly designed and implemented by Victor W H Wu (邬伟雄).

This project uses a self built CNC Mechanism, driven by *stepper motors*. It uses a *laser scanner* - Keyence LJ V7200 to scan welding grooves. This will help CNCWELD to locate welding grooves relative to the CNC machine without time consuming setup and alignment time.

The whole project is implemented in the Robot Operating System (ROS). As the laser scanner scans welding grooves, a point cloud showing the welding groove is shown in the RViz (ROS Visualizer). At the same time, *way-points* of the welding path is also shown.

When the scanning is completed, users can instruct CNCWELD where to begin and where to end.

After the way-points of the welding path is confirmed, the CNC Mechanism will start welding from either end as instructed by users.

## ROS

This project is built on ROS. ROS has its structures. Every project has a *work space*. In side the work space, there is a `src` folder. 

ROS projects can be built using either `catkin_make` or `catkin build`. CNCWELD chose to use `catkin_make`. Once the project has been built once, catkin will generate two more folers, `build` and `devel`. 

Before [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) can be used, it must be installed. CNCWELD is built on *ubuntu 18.04* and *ROS Melodic*.

Each model of a ROS project is called a *package* inside the top level `src` of the project. Every package must have its own `CMakeLists.txt` and `package.xml` detailing how the package should be built and all its *dependences*. Each package should also has its own `src` folder, if it is in C++, or `script` folder if it is in *python*.

## Modules

There are four main modules in CNCWELD. They are:
1. cncweld_description, describes the structure of the CNC mechanism
2. cncweld_support, provides setups and launch files
3. keyence_experimental, the laser scanner driver
4. cncweld_core, privides the core functionalities

### 1. CNCWELD Description

To control the CNC mechanism, it is necessary to build a model of it. ROS uses the Universal Robot Description Format (URDF) to describe robots. The CNC mechanism used in this project is a kind of robot, with 3 *prismatic* joints for the X, Y, and Z axes and a *rovolute* joint for the *A* axis. In CNC terms, the *A* axis is the rotating axis that rotates about the X axis. Therefore the *welding torch* can swing from side to side of a welding groove.

|![A picture of RViz showing the CNC mechanism](images/CNC%20Mechanism%20visualized%20in%20RViz%20(20%20May%202022).png) |
| :--: |
| *CNCWELD as shown in RViz*  |

For a good practice, the CNC mechanism should be put in a description folder, and all the other end effectors should be put in the cncweld_support folder. 

The cncweld_support should then provide launch files that setup for different end effectors. It is common that welding machines have welding torches of different shapes.
Usually, there are at least *straight* and *bended* torches. When they are used, different urdf (or xacro) files are needed and specified by different *launch* files.

### 2. GRBL on Arduino and the CNC mechanism

GRBL is one of the very popular *firmware* running on *Arduino* *UNO* or *Mega 2650* for driving CNC Devices, especially hobbist 3D printers.

CNCWELD uses GRBL to drive its CNC mechanism.

Pictures of the stepper motors, drivers, lead screws, limit switches and the Arduino should be inserted here.

![Pinout](images/grbl-mega-5x%20pinout%20.jpeg) of the Arduino Mega 2560 for Grbl Mega 5X is 
different from other grbl.


