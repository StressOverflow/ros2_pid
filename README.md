# ROS 2 🐢 PID CONTROLLER 🎛️

[![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-violet)](https://releases.ubuntu.com/22.04/)
[![distro](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![main](https://github.com/StressOverflow/ros2_pid/actions/workflows/colcon.yaml/badge.svg?branch=dev)](https://github.com/StressOverflow/ros2_pid/actions/workflows/colcon.yaml)

## Goal 🎯

Until now, we have been using *bang-bang* or *Proportional* controllers for most of our applications. But soon we realized that eventually we will need more fine grained control. A [**PID**](https://en.wikipedia.org/wiki/PID_controller) is the answer!

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/commons/3/33/PID_Compensation_Animated.gif" />
</p>

We *have borrowed* the [PID Controller](https://github.com/fmrico/book_ros2/blob/a489bb519fb62d3bd22a0c51c3c638b19bf77d5f/br2_tracking/include/br2_tracking/PIDController.hpp#L24) of the [ROS 2 Book repo](https://github.com/fmrico/book_ros2). It did the job for a while, but we wanted to take it to the next level.

We are pretty sure that we are not the first ROS 2 developers on earth who needed a PID Controller, but we are here to learn as well, aren't we?

That is why we decided to make our own implementation of a PID Controller specifically meant to work with ROS 2. However, [**It's dangerous to go alone**](https://www.youtube.com/watch?v=z1w8FCtpVXM), so we thought that maybe it was a good idea to *borrow* again some foreign code...

### The Inspiration 💡

Some members of our group had been tweaking with microcontrollers for a while now, more precisely with [Arduino](https://www.arduino.cc/) boards. They had successfully used the fantastic [PID Library](https://playground.arduino.cc/Code/PIDLibrary/) a bunch of times, and thanks to the magnificent idea of **Open Source**, we can actually take a look on how it is implemented!

Not only that, [the source code is also in GitHub](https://github.com/br3ttb/Arduino-PID-Library/). But wait, there is more!

His author [Brett Bauregard](https://github.com/br3ttb), [has a fantastic blog](http://brettbeauregard.com/blog/) where he explains all the details of the implementation, from the code to the math. In Spain 🇪🇸 we would say *Canelita en rama*.

**So actually what we are trying to do is port his library to ROS 2**. Cool, huh?

## Development 🧑‍💻

Apparently, his blog has a lot of theory about PID Controllers that were not implemented on the final version of the library seeking for simplicity, however we aim to get them working on our port.

That means that this development will not be a piece of cake! If you are in a hurry, you can always help us to do it... 😉

## Installation 💾

### Main requirements ✅

1. First of all, we need [Ubuntu](https://ubuntu.com/). It is a Linux Debian-based operating system, A.K.A Linux Distro. We are working with the **[22.04 Jammy Jellyfish 🪼](https://releases.ubuntu.com/22.04/)** version. You might be already familiar with it, however [here](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) you have a step-by-step tutorial on how to install it on your machine.

2. [ROS](https://www.ros.org/). ROS stands for *Robot Operating System*, and it is the **middleware** that the vast majority of robots out there use nowadays, and so are we! We are focusing on the latest stable release, which is **[ROS 2 Humble Hawksbill 🐢](https://docs.ros.org/en/humble/index.html)**. From there you can navigate through all the documentation, but [here](https://docs.ros.org/en/humble/Installation.html) is a shorcut to the installation page.

> **FUN FACT**
>
> Did you know that every ROS distribution is named alphabetically? Not only that, but it is always a turtle species which starts with the current letter. Hawksbill is the colloquial name for the turtle species *Eretmochelys imbricata*. [Here](https://en.wikipedia.org/wiki/Hawksbill_sea_turtle) you can learn more about this animal if you feel curious!

This is supposed to be used as a library, is a light-weight package with everything set to install and run on your applications.

#### `package.xml`

As a generic dependency:

```xml
<depend>ros2_pid</depend>
```
Or as a build dependency:

```xml
<build_depend>ros2_pid</build_depend>
```

#### `CMakeLists.txt`

You will need to add this package as any other package you probably had already use.

```cmake
find_package(ros2_pid REQUIRED)

# ...

ament_target_dependencies(<TARGET_NAME> 
  ros2_pid
)

# ...
```

And you are set! Now you will be able to use it on your sources!

```c++
#include "ros2_pid/PIDController.hpp"

ros2_pid::PIDController myPID;

myPID.init(node);
myPID.set_pid(KP, KI, KD);

// ...

double newReference = 3.14159265358979323846

myPID.set_reference(newReference);
myPID.compute();

double newOutput = myPID.get_output();

// ...
```

> Yup, lack of doc... We are on it!

## About

This is a project made by the [StressOverflow], a student group from the [Universidad Rey Juan Carlos].
Copyright &copy; 2023.

Maintainers:

* [Carlos Escarcena]
* [Arantxa García]
* [Diego García]
* [Teresa Ortega]

## License

[![License](https://img.shields.io/badge/License-Apache_2.0-yellowgreen.svg)](https://www.apache.org/licenses/LICENSE-2.0)

This work is licensed under a [APACHE LICENSE, VERSION 2.0][apache2.0].

[apache2.0]: https://www.apache.org/licenses/LICENSE-2.0

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[StressOverflow]: https://github.com/StressOverflow
[Carlos Escarcena]: https://github.com/cescarcena2021
[Arantxa García]: https://github.com/arantxagb
[Diego García]: https://github.com/dgarcu
[Teresa Ortega]: https://github.com/mtortega2021
