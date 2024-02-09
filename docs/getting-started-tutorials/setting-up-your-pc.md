# Setting up your PC


The **first step** in getting started on the competition is to get access to a computer with the right system requirements, and set it up with the right operating system, software and environment to run the competition packages.


<!-- This guide helps you setup your computer for running the competition environment locally and developing the code.

You can use local Computer/Laptop or Virtual Machine inside your computer or any cloud platform like [Google GCP](https://cloud.google.com/free){target=_blank}, [Amazon AWS](https://aws.amazon.com/free/){target=_blank}, [Microsoft Azure](https://azure.microsoft.com/en-us/free/){target=_blank}, [Digital Ocean](https://try.digitalocean.com/freetrialoffer/){target=_blank}, etc (All Cloud providers have some free trial plan which you can make use of). -->


## System requirements

You will need a computer that has all (or at least most) of these specifications:
    
- A dedicated [GPU](https://en.wikipedia.org/wiki/Graphics_processing_unit){target=_blank},
    - Nvidia cards tend to work well in Ubuntu
- A CPU that is at least an Intel i5, or equivalent,
- At least 4GB of free disk space,
- At least 8GB of RAM,
<!-- - [Ubuntu Focal Fossa](https://releases.ubuntu.com/focal/){target=_blank} installed. -->

## Operating System (OS)

We will be using **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** operating system (OS) which is a flavor of [Linux](https://en.wikipedia.org/wiki/Linux){target=_blank}. We know that some participants may have limited or no experience using Ubuntu, so here is a guide on different ways of setting up an operational Ubuntu Focal instance for this competition

If you have a Windows PC (or any other operating system different from Ubuntu Focal 20.04), here are three (3) options to explore:

- Option 1 (Recommended): **Dual-Boot**: Install **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** in a dual-boot alongside your Windows OS.
- Option 2: **Using Docker:** Run **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** in a Docker container on your native Windows OS.
- Option 3: **Using Virtual Machine:** Run **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** in a Virtual Machine (VM) on your native Windows OS.

=== "Dual-Boot (Recommended)"
    - Here is a [good guide](https://medium.com/linuxforeveryone/how-to-install-ubuntu-20-04-and-dual-boot-alongside-windows-10-323a85271a73){target=_blank} you can follow to install **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** in a dual-boot alongside your Windows OS.
=== "Using Docker"
    - We have provided a [detailed guide](../getting-started-tutorials/setting-up-with-docker.md) on installing Docker on your PC and setting up the right Docker container to run the entire competition. 
    - If you are concerned about the dual-boot option, we recommend considering this option.

=== "Using Virtual Machine"
    - Here is a [good guide](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview){target=_blank} you can follow to install VirtualBox on your PC and run Ubuntu. 
    - **NOTE:** The above guide focuses on Ubuntu 22.04 whereas we need you to install Ubuntu 20.04, you would need to modify some steps to reflect this.
    - If you prefer video tutorials, consider this: [Installing Ubuntu 20.04 on VirtualBox](https://www.youtube.com/watch?v=x5MhydijWmc){target=_blank}




<!-- ## Operating System
If not already installed, Install **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** on the system by following [this guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview){target=_blank}.

!!! note
    It is highly recommended to install [Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank} version of Ubuntu due to [ROS (Noetic)](http://wiki.ros.org/noetic){target=_blank} dependency. -->

## Installing ROS
!!! note
    If you followed the **Using Docker** option above, please SKIP this step.

Once you have a fresh **[Ubuntu Focal (20.04)](https://releases.ubuntu.com/focal/){target=_blank}** installation, the next step is to install ROS. We are using the [ROS Noetic](http://wiki.ros.org/noetic) distribution for this competition. You can install ROS Noetic by following [this guide](http://wiki.ros.org/noetic/Installation/Ubuntu){target=_blank} and install `ros-noetic-desktop-full` in the step `1.4` of the guide.

If you prefer video instructions, you can follow this video: 

- [Installing ROS Noetic on Ubuntu 20.04](https://www.youtube.com/watch?v=ZEfh7NxLMxA){target=_blank}

<!-- ## Next Steps

To set up your workspace, there are two available options. You can either use Docker or manually configure your workspace. If you prefer to configure your workspace manually, follow the instructions provided [here](../getting-started-tutorials/setting-up-your-workspace.md). Alternatively, if you would like to set up your workspace using Docker, follow the instructions provided [here](../getting-started-tutorials/setting-up-your-workspace-using-docker.md). -->
