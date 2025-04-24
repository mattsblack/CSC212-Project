# Enhancing Roomba Navigation: A Comparative Study of Pathfinding Algorithms

## Objectives

This project aims to enhance the efficiency of Roomba robots in real-world settings by comparing the performance of different algorithms. The research uses pre-existing pathfinding algorithms for the iRobot Create 3 to implement a coverage measuring system. A comparative analysis will be conducted, measuring and evaluating key factors such as coverage, time consumption, and overall efficiency of each algorithm.

A primary focus for this project is simultaneous localization and mapping (SLAM), a technique that enables a robot to create and update a map of an unknown environment while simultaneously keeping track of its position. We will utilize LiDAR technology installed on iRobot Create 3 and a Raspberry Pi 4 running ROS2 to implement a system that creates a map of the room covered and records the robot’s path. These maps will be generated using software designed for mapping and visualization.

The generated maps are then analyzed, and the data recorded is compared to identify algorithms that provide more comprehensive coverage. We anticipate that this research will contribute to creating a more systematic approach to analyzing pathfinding strategies, reaching beyond the algorithms used in the comparative analysis in this study. The results of this project will potentially be used in more informed decision-making in the selection of algorithms for autonomous consumer-grade vacuuming robots used in diverse environments.

## Getting started

You will need:

- iRobot's Create3 robot
- Raspberry Pi 4
- Ubuntu 22.04 (either on a virtual or physical machine)
- ROS2 Humble
- Slamtec RPLiDAR A1M8
- USB-C to USB-C cable
- USB Micro B to USB A cable
- Ethernet to USB-C adapter
  
---

## Software setup

### Raspberry Pi setup

First, follow the [Create3 instructions](https://iroboteducation.github.io/create3_docs/setup/pi4humble/) to set up Ubuntu on the Raspberry Pi.

Next, follow the [Create3 LiDAR SLAM example setup instructions](https://github.com/iRobotEducation/create3_examples/tree/humble/create3_lidar_slam) to set up the robot and the Raspberry Pi.

### Computer setup

>[!IMPORTANT]
>if you are using a virtual machine to run Ubuntu 22.04, make sure to set the networking mode to Bridged so the robot can communicate with your VM.
>
<details>
  <summary>Important info for Apple Silicon Mac users (click to expand)</summary>
  <br>
If you are using Apple Silicon, you must install Ubuntu 22.04 in a virtual machine. 

Download the 64-bit ARMv8 version of Ubuntu 22.04 server [here](https://cdimage.ubuntu.com/releases/22.04/release/).

Set up the virtual machine using your VM Software (this example uses UTM).

Make sure to set the networking mode to **Bridged (Advanced)**, and the Bridged Interface to **Automatic**. 

This will allow the virtual machine to communicate with the robot.
  <details>
    <summary>UTM setup instructions (click to expand)</summary>
  
  Click on "Create a New Virtual Machine"
  
  <img width="352" alt="image" src="https://github.com/user-attachments/assets/e07c7d2f-6737-49d6-8120-1a7a48fea08f" />
  
  Then click "Virtualize"
  
  <img width="444" alt="image" src="https://github.com/user-attachments/assets/710ea1f9-b1c6-4111-a7aa-3643e5c06a1b" />

  
  Then under "Preconfigured" click on "Linux"
  
  <img width="438" alt="image" src="https://github.com/user-attachments/assets/f9d6fbe0-a2cc-4b3e-8b39-2aed1c0b77b2" />

  Then leave Apple Virtualization unchecked, as we are using QEMU. Select the Ubuntu 22 server ISO that you downloaded earlier.
  
  <img width="442" alt="image" src="https://github.com/user-attachments/assets/ec304fda-77a9-4383-88d7-0f9660f6e0d7" />

  Then under hardware choose the amount of CPU cores and memory you want to allocate to the VM. I kept the default settings.

  <img width="444" alt="image" src="https://github.com/user-attachments/assets/eee14698-61a2-4fa1-9722-92e47434186d" />

  Then choose the amount of storage you want to use for the VM, and if you want to create a shared directory with your computer's OS and the VM.

  After setup it should look like this

  <img width="440" alt="image" src="https://github.com/user-attachments/assets/f3e0b525-3641-43a5-b571-b4d47ad4f84d" />

  Next, start the VM and follow the setup instructions. After it completes the install, shutdown the VM and remove the ubuntu 22 ISO from the CD/DVD drive.

  <img width="639" alt="image" src="https://github.com/user-attachments/assets/04de0b73-fed8-4fce-99c8-4a432c5fa910" />

  Next, boot into the VM run `sudo apt update && sudo apt upgrade` and then `sudo apt install ubuntu-desktop`. Then type reboot and load the VM. It should now load the desktop login page.

  Next, shut down the VM, right click and select "Edit" 
  
  <img width="297" alt="image" src="https://github.com/user-attachments/assets/8e53d200-d1b8-43f3-8d41-11c21a2b506e" />

  Under "Devices" click on "Network"

  <img width="795" alt="image" src="https://github.com/user-attachments/assets/82e8d325-8aae-485a-974d-c663087c8a68" />

  Change the Network Mode to "Bridged (Advanced)" and the Bridge Interface to "Automatic". This will allow the VM to communicate with the robot.

  <img width="788" alt="image" src="https://github.com/user-attachments/assets/f0543bfe-e286-4761-888d-6314b1fca4be" />

  </details>
</details>

## Project structure

This project has two approaches: one records data in a ROSbag file and displays it in RViz during playback, while the other displays real-time data in RViz using LiDAR, and other sensor data.

Setup for the ROSbag approach is located in these folders:

- [create3_spiral_coverage](./create3_spiral_coverage/README.md)

- [create3_straight_coverage](./create3_straight_coverage/README.md)

- [rosbag_path_overlay](./rosbag_path_overlay/README.md)

Setup for the real-time approach is located in these folders:

- [create3_mapper](./create3_mapper/README.md)

- [path_publisher](./path_publisher/README.md)

## References

### Publications

1. R. Gunning, “A performance comparison of coverage algorithms for simple robotic vacuum cleaners,” Dissertation, 2018, <https://diva-portal.org/smash/get/diva2:1213970/FULLTEXT02.pdf>
2. Hasan, Kazi Mahmud, et al. “Path planning algorithm development for autonomous vacuum cleaner robots.” 2014 International Conference on Informatics, Electronics &amp; Vision (ICIEV), May 2014, pp. 1–6, <https://www.researchgate.net/publication/269297110_Path_planning_algorithm_development_for_autonomous_vacuum_cleaner_robots>
3. Zheng, Kuisong, et al. “Performance metrics for coverage of cleaning robots with MoCap system.” Lecture Notes in Computer Science, 2017, pp. 267–274, <https://doi.org/10.1007/978-3-319-65298-6_25>
4. Zhao, Shengmin, and Seung-Hoon Hwang. “Complete Coverage Path Planning Scheme for autonomous Navigation Ros-based robots.” ICT Express, vol. 10, no. 1, Feb. 2024, pp. 83–89, <https://doi.org/10.1016/j.icte.2023.06.009>
5. Gabriely, Y., and E. Rimon. “Spanning-tree based coverage of continuous areas by a mobile robot.” Proceedings 2001 ICRA. IEEE International Conference on Robotics and Automation (Cat. No.01CH37164), vol. 2, pp. 1927–1933, <https://doi.org/10.1109/robot.2001.932890>

### Other Resources

1. Layton, Julia. “How Robotic Vacuums Work.” HowStuffWorks, HowStuffWorks, 3 Nov. 2005, <https://electronics.howstuffworks.com/gadgets/home/robotic-vacuum.htm>
2. “Navigation Technology.” iRobot, 23 Aug. 2024, <https://homesupport.irobot.com/s/article/31056#:~:text=Robots%20that%20use%20iAdapt%C2%AE%201.0%20Navigation%20Technology%20utilize%20a,on%20a%20single%20battery%20charge>
3. “Why Your Roomba Takes a Weird Path to Keep Your Floors Clean.” CNET, <www.cnet.com/home/kitchen-and-household/this-is-why-your-roombas-random-patterns-actually-make-perfect-sense/>
4. Olson, Jon, et al. “Simple Mapping and Path-Planning with the Roomba”, 2008, pp. 1–4., <https://joosm.github.io/RIP2014/material/pastprojects/RIP08_JohnJonIvan.pdf>
5. Kapoutsis, Athanasios. “DARP: Divide Areas Algorithm for Optimal Multi-Robot Coverage Path Planning”, Medium, 1 Sep. 2021, <https://medium.com/@athanasios.kapoutsis/darp-divide-areas-algorithm-for-optimal-multi-robot-coverage-path-planning-2fed77b990a3>
