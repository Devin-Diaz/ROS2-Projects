# Overview Of ROS

**ROS 2 - Robot Operating System 2 is an opensource framework designed to support the development of robotic systems. It is the successor of ROS 1, created to address some of the limitations and evolving needs in robotics, such as real time control, multi-robot systems, and secure communications.

**ROS 2 provides a structured communication layer above the operating system, which allows robotic components like sensors, actuators, controllers, and algorithms to communicate with each other in a modular, flexible way.\

**Resource:
https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy

---
##### Architecture and Key Components:

###### Nodes
- The smallest unit of processing in ROS 2.
- Each node is a process that performs computation.
- Nodes can be written in various languages but mainly Python and C++.
- Any program that has access to ROS functionalities and communications. 
- Nodes can be graphical interfaces, communication among each other, display logs, interact with hardware, or host a server.
###### Topics
- Used for unidirectional, async communications.
- Nodes can publish messages to a topic.
- Other nodes can subscribe to that topic to receive messages. 
- Basis for sensor data streams, command dissemination, and general inter-node communication.
###### Services
- Provides synchronous, bidirectional communication mechanism.
- A service call involves a request from one node and a response from another node.
- Useful for tasks that require confirmation or a response, such as querying a sensor or setting a parameter.
###### Actions
- Used for async, long-duration tasks that require feedback and the possibility of cancellation.
- Actions are more complex than services.
- Useful for tasks like moving a robot arm to a specific position or performing a navigation task.
---
##### ROS 2 Communication Middleware

**Middleware is software that provides services to software apps beyond those available from the operating system. Also known as "software glue".

ROS 2 uses Data Distribution Service (DDS) as its communication middleware. DDS provides the following advantages such as:

**Quality of service**: Control over communication parameters such as reliability, durability, and latency.

**Security**: Built in mechanisms to ensure secure communication between nodes. 

**Real-Time Performance**: Support for real-time communication, which is crucial for robotics.

---
##### Development Tools and Ecosystem
- **rviz2 - A 3D visualization tool for debugging and viewing sensor data, robot models, and more.  
- **Gazebo - A powerful 3D robot simulator that integrates seamlessly with ROS 2.
- **rqt - A Qt based framework for developing graphical interfaces in ROS 2.
- **rosbag - Tools for recording and playing back ROS 2 messages, useful for testing and debugging
- **Colcon - A command line tool to build and manage ROS 2 work spaces. 
---
##### Getting Started:
**It's important we have ROS 2 environment setup on every terminal emulator we use. We can ensure ROS 2 is setup automatically upon opening the terminal by adjusting our bashsc file and adding the following line at the bottom of the script

```
source /opt/ros/humble/setup.bash
```

###### Running talking and listening Nodes

**Using the following commands
```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

Running the first command will begin a script where nodes are spitting out information or "talking", by running the second script, it will actually intercept that information from the talking node and let us know what it's intercepting. Recall a Node is any computation done in ROS 2. 

We can display a graph of the process occurring with these two scripts by running
```
rqt_graph
```

---

**To begin writing our own Nodes we need to create and setup our own workspace. Information about this can be found here: [[Create and Set Up ROS 2 Workspace]]



