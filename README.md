# Enhancing Roomba Navigation: A Comparative Study of Pathfinding Algorithms
## Objectives
The goal of this project is to compare two pathfinding algorithms and analyze their coverage and time efficiency.
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
### Software setup

#### Raspberry Pi setup
First, follow the [Create3 instructions](https://iroboteducation.github.io/create3_docs/setup/pi4humble/) to set up Ubuntu on the Raspberry Pi.

Next, follow the [Create3 LiDAR SLAM example setup instructions](https://github.com/iRobotEducation/create3_examples/tree/humble/create3_lidar_slam) to set up the robot and the Raspberry Pi.


#### Computer setup

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


