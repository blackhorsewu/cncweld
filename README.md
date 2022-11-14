# <p align="center"> Chinese National Engineering Research Centre for Steel Construction (Hong Kong Branch) </p>
# <p align="center"> 国家钢结构工程技术研究中心 （香港分中心）</p>

# CNCWELD

## Introduction

This project tries to use a CNC Mechanism with 3 degrees of freedom (dof) to do autonomous Gas Metal Arc Welding (GMAW).

It was mainly designed and implemented by Victor W H Wu (邬伟雄).

This project uses a self built CNC Mechanism, driven by *stepper motors*. It uses a *laser scanner* - Keyence LJ V7200 to scan welding grooves. This will help CNCWELD to locate welding grooves relative to the CNC machine without time consuming setup and alignments.

The whole project is implemented in the Robot Operating System (ROS). As the laser scanner scans welding grooves, a point cloud showing the welding groove is displayed in the RViz (ROS Visualizer). At the same time, *way-points* of the welding path is also shown.

When the scanning is completed, users can instruct CNCWELD where to begin and where to end.

After the way-points of the welding path is confirmed, the CNC Mechanism will start welding from either end as instructed by users.

## ROS

This project is built on ROS. ROS has its structures. Every project has a *work space*. Inside the work space, there is a `src` folder. 

ROS projects can be built using either `catkin_make` or `catkin build`. CNCWELD chose to use `catkin_make`. Once the project has been built once, catkin will generate two more folders, `build` and `devel`. 

Before [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) can be used, it must be installed. CNCWELD is built on *ubuntu 18.04* and *ROS Melodic*.

Each model of a ROS project is called a *package* inside the top level `src` of the project. Every package must have its own `CMakeLists.txt` and `package.xml` detailing how the package should be built and all its *dependencies*. Each package should also has its own `src` folder, if it is in C++, or `script` folder if it is in *python*.

## Modules

There are four main modules in CNCWELD. They are:
1. cncweld_description, describes the structure of the CNC mechanism
2. cncweld_support, provides setups and launch files
3. keyence_experimental, the laser scanner driver
4. cncweld_core, provides the core functionalities

### 1. CNCWELD Description

To control the CNC mechanism, it is necessary to build a model of it. ROS uses the Universal Robot Description Format (URDF) to describe robots. The CNC mechanism used in this project is a kind of robot, with 3 *prismatic* joints for the X, Y, and Z axes. 

The supplier of the CNC mechanism provided a 3D drawing of the parts in 
SolidWorks format in [this](SolidWorks-Drawings/FSL80X1000Y500Z200-L.STEP) file.

|![A picture of RViz showing the CNC mechanism](images/CNC%20Mechanism%203.png) |
| :--: |
| *CNCWELD as shown in RViz*  |

For a good practice, the CNC mechanism should be put in a description folder, and all the other end effectors should be put in the cncweld_support folder. 

The cncweld_support should then provide launch files that setup for different end effectors. It is common that welding machines have welding torches of different shapes.
Usually, there are at least *straight* and *bended* torches. When they are used, different urdf (or xacro) files are needed and specified by different *launch* files.

### 2. GRBL on Arduino and the CNC mechanism

GRBL is one of the very popular *firmware* running on *Arduino* *UNO* or *Mega 2650* for driving CNC Devices, especially hobbyist 3D printers.

CNCWELD uses GRBL to drive its CNC mechanism.   

### 2.1 Install Arduino IDE

To install the Arduino IDE in Ubuntu 20.04, just do  
`sudo apt-get install arduino`  

### 2.2 Download GRBL for Arduino Mega 2560

Download [this](https://github.com/gnea/grbl-Mega) particular version of GRBL, Download ZIP.

Then unzip this file. 

### 2.3 Flash (install) GRBL onto Arduino Mega 2560

In the Arduino IDE, click Tools > Board > Arduino Mega 2560.

Sketch > Import Library ... > Add Library > then choose the downloaded GRBL file.

Then, Sketch > Import Library > under contributed choose grbl.

Then, Sketch > Verify/Compile.

Then, File > Upload.

### 2.4 Check GRBL is uploaded onto Arduino Mega 2560

Tools > Serial Monitor.

Input `$$` to the Monitor.

### 2.5 Wiring of Limit Switches   

[This](https://github.com/gnea/grbl/wiki/Wiring-Limit-Switches) web page describes how to connect the **limit switches** to Arduino.

The Limit Switches used are NO (Normal Open) proximity switches. 
It has 3 wires output,   
Brown (positive power),   
Black (Normal Open),    
Blue (negative power or common or ground).   

The ground (GND) should be connected to Pin A16 (after A15).

Pictures of the stepper motors, drivers, lead screws, limit switches and the Arduino should be inserted here.

|![Pinout](images/grbl-mega-5X%20pinout.jpeg)|
| :--: |
| Pinout of the Arduino Mega 2560 for Grbl Mega 5X |

The *Enable Spindle* pin 6 (`M3` and `M5`) are used to switch on and off the welding torch.
The *Enable Coolant* pin 8 (`M8` and `M9`) are used to switch on and off the laser scanner.


A [video](videos/CNC%20Scanning%2022%20June%202022-1.mp4) showing how a welding groove
is scanned by the laser scanner.

### 3. Universal G-Code Sender (UGS)

The UGS is downloaded from [here](https://winder.github.io/ugs_website/download/). 

https://github.com/SeungBack/open3d-ros-helper/issues/6
