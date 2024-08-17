# Create and Set Up ROS 2 Workspace

**Recall that sourcing is a process in Unix like operating systems where a script is executed in the current shell environment rather than a sub shell.  This means variables and functions defined in the script become available in the current shell session.

**We do this in ROS 2 for certain scripts to set up environments correctly so that the ROS 2 tools and packages are accessible in terminal sessions. 

**setup.bash** - This is sourced in /opt/ros/humble so the shell knows where to find ROS 2 executables and packages installed under ROS 2.  

**colcon-argcomplete.bash** - **colcon** is a command line tool used to build ROS 2 packages. The arg complete script enhances productivity by providing auto-completion for colcon commands. 

## Creating First ROS 2 Node

---
##### Workflow For Setting Up Project:

1. Create a root directory (name of our choice), cd to our root and create a source folder (where node code will reside)
```
mkdir ros2_ws
cd ros2_ws
mkdir src
```

2. Create colcon build within root directory
```
colcon build
```

3. After our colcon build, new directories will be in our root, /build, /install, and /log. What's important is there is a setup.bash file we will need to source in our /install directory. We will cd to our install directory, copy the setup.bash file, and source to the path leading up to the install directory.
```
source ~/ros2_ws/install/setup.bash
```

4. Lastly we add this to our bashrc file to ensure the environment runs into no complications upon running this project on different terminal emulators. 
```
# paste path in here
gedit ~/.bashrc
# source the bashrc file in any new terminal windows we open
# do this via source .bashrc if in home directory
```

---

##### Create A ROS 2 Python Package

**Nodes will be written in Python packages.**
**Underscores for package names is the norm.**
**Have choice of Python or C++**

1. cd to our source folder with our root directory.  Create our package, with a build system using Python. Recall colcon is our build tool (chef) and ament is build system (recipe).  Then add dependencies for this particular package.   
```
cd src
ros2 pkg create my_robot_controller --build-type ament_python --dependencies rclpy
```

2. Run colcon build command again to build the package. Recall **YOU MUST BE IN ROOT DIRECTORY**
```
cd ..
colcon build
```

3. If we run into an error to package deprecation, install python3-pip and check within piplist that the package setuptools is on version 58.2.0

**We are now ready to write the code for our first node**

---

##### Writing Python Code For Node

**Directories with an init file contained in them indicate that the directory is marked as a Python package. This initializes the package when it's imported and executes code within the package.**

1. Within the package we created, there is going to be another directory with the same name as the package. We cd to that directory and it should contain a **init** file. We will create an executable Python file where node code will go.
```
touch first_node.py
chmod +x first_node.py
```

2. If you are using Vs code as IDE, ensure you have ROS extension by Microsoft installed as it will allow you to autocomplete syntax for better workflow.

3. Start coding first node!
---

**Any time you change your setup python file, you must build your environment again and source it.  Recall if the source path is already added on the bashrc file you only have to do 
```
source ~/.bachrc in root folder of project
```

---

##### Running Code

###### Running Python File Directly:
- Running the Python script does whatever is written inside it.
- It's like double clicking a program on your computer to start it
- **Must be in package directory where file that contains node is
```
./first_node
```

###### Running ROS 2 package
- We are asking ROS 2 to run a specific node that's part of a larger package. ROS 2 manages the node and connects it to other parts of your robot system, like sensors or other nodes.
- It's like launching an app on your phone that only starts the app but connects to the internet, GPS, and other apps.
- **Must add function that's running node in setup.py -> entry_points
- **Must be in root directory
```
setup.py
-------
# file name DOES NOT HAVE EXTENSION

entry_points={
'console_scripts': [
	"executable_name = package_name.file_name:function_name"
],
```

**After creating an executable name, we must colcon build again in the root folder and source it again. 

Then lastly run the package
```
ros2 run package_name
```

