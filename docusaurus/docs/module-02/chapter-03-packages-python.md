---
title: "Chapter 3: Building ROS 2 Packages with Python"
---

# Chapter 3: Building ROS 2 Packages with Python


-   **Learning Objectives**:
    -   Understand the standard structure of a ROS 2 Python package.
    -   Identify the key files in a package: `package.xml` and `setup.py`.
    -   Declare package dependencies and entry points.
    -   Use `colcon` to build a package and source the workspace to make it available.
    -   Create a launch file to run multiple nodes at once.

## Introduction 

In the world of software engineering, modularity and reusability are paramount. We don't write complex applications as a single, monolithic file; we break them down into organized, maintainable, and distributable units. In ROS 2, the fundamental unit of organization is the **package**. A package is a directory containing your node's source code, configuration files, launch files, and a manifest file (`package.xml`) that describes it all.

This chapter will guide you through the process of creating and building a proper ROS 2 package in Python. While we created a basic package in the last chapter, here we will dive deeper into the "why" behind the structure. You will learn the specific roles of `package.xml`, where you declare metadata and dependencies, and `setup.py`, where you define how your Python code should be installed and what executables it provides. Understanding these files is the key to creating portable and shareable robotics software.

Furthermore, we will introduce a powerful tool for managing complex applications: **launch files**. Running each node in a separate terminal is fine for simple examples, but a real robot might have dozens of nodes that need to be started in a coordinated way. ROS 2 launch files allow you to define a multi-node application in a single Python script, giving you programmatic control over how your system starts, stops, and is configured. By the end of this chapter, you will be able to structure your code like a professional robotics developer.

---

## Main Content 

### The Anatomy of a ROS 2 Package

A ROS 2 package is more than just a folder with code. It's a standardized directory structure that allows ROS 2's build tools, like `colcon`, to automatically find, build, and install your software. A minimal Python package (`ament_python`) has the following key components:

```
my_python_pkg/
├── package.xml               # The package manifest
├── setup.py                  # Python's setup script for installation
├── setup.cfg                 # Configuration for the setup script
├── resource/
│   └── my_python_pkg         # A marker to identify the package
└── my_python_pkg/              # The actual Python source code lives here
    ├── __init__.py           # Makes this directory a Python package
    └── my_node.py            # Your node's source file
```

Let's examine the two most important files in detail.

#### `package.xml`: The Package Manifest

This is an XML file that contains metadata about your package. It answers questions like:
-   What is the name of this package?
-   What version is it?
-   Who is the author and maintainer?
-   What license is it released under?
-   What other ROS 2 packages does it depend on?

A typical `package.xml` for a simple Python node might look like this:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_python_pkg</name>
  <version>0.0.0</version>
  <description>A simple Python package for ROS 2 examples.</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
The most important tags for now are the `<depend>` tags. Here, we are telling the ROS 2 build system that our package **depends on** `rclpy` (the ROS 2 Python client library) and `std_msgs` (which provides the standard message types like `String`). This ensures that if someone tries to build our package, the system will first make sure these dependencies are available.

#### `setup.py`: The Python Build Script

This is a standard Python `setuptools` script. It tells the build system how to handle your Python source code. Its primary job is to define the `entry_points` that create your executables.

```python
from setuptools import setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_python_pkg.talker:main',
            'listener = my_python_pkg.listener:main',
        ],
    },
)
```
The key section is `entry_points`. The `'console_scripts'` dictionary creates a mapping.
- **Key (`talker`):** This is the name of the command-line executable you want to create. When you run `ros2 run my_python_pkg talker`, ROS 2 looks for this key.
- **Value (`my_python_pkg.talker:main`):** This tells ROS 2 what to execute. It means "in the `my_python_pkg` Python module, find the file `talker.py`, and inside that file, run the function named `main`."

### The Build Process with `colcon`

As we saw in the last chapter, `colcon` is the tool used to build a workspace. When you run `colcon build`, it performs several steps for each Python package:
1.  It finds all packages in the `src` directory by looking for `package.xml` files.
2.  It creates a `build` directory for intermediate files.
3.  It creates an `install` directory where the final, installed version of your package will be placed.
4.  It runs the `setup.py` script for your package. This script copies your Python source code into the `install` directory and, crucially, creates the executable files defined in `entry_points`.
5.  It generates `setup.bash` (and other shell files) in the `install` directory.

:::warning[Sourcing Your Workspace]
This last step is critical. The ROS 2 system only knows about the packages that are in the paths defined by your environment variables. Running `source install/setup.bash` **overlays** your new workspace on top of your existing ROS 2 environment. This updates the environment to make it aware of your new `my_python_pkg` package and the `talker` and `listener` executables you created. You must do this in every new terminal where you want to use your package.
:::

### Managing Complexity with Launch Files

Running each node in a separate terminal is tedious and doesn't scale. A real robot might require 10, 20, or even more nodes. ROS 2 **launch files** solve this problem. A launch file is a script (usually in Python) that can start and configure multiple nodes at once.

Let's create a launch file to start our talker and listener nodes simultaneously.

**Instructions:**
1.  Create a `launch` directory inside your package: `mkdir my_python_pkg/launch`.
2.  Create a new file: `my_python_pkg/launch/talker_listener_launch.py`.

**Code:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for starting the talker and listener nodes.
    """
    return LaunchDescription([
        Node(
            package='my_python_pkg',
            executable='talker',
            name='my_talker_node' # Optional: Remap the node name
        ),
        Node(
            package='my_python_pkg',
            executable='listener',
            name='my_listener_node'
        ),
    ])
```

**How it Works:**
-   We import the necessary modules from the `launch` and `launch_ros` libraries.
-   The `generate_launch_description` function is the main entry point that ROS 2 looks for.
-   This function returns a `LaunchDescription` object, which is a list of actions to perform.
-   The `Node` action is used to start a node. We specify the `package` and `executable` names that we defined in our `setup.py`. We can also optionally provide a new `name` for the node.

**Running the Launch File:**
After building your package with `colcon build` and sourcing the `install/setup.bash` file, you can run the launch file with a single command:

```bash
ros2 launch my_python_pkg talker_listener_launch.py
```
You will now see the output from both the talker and the listener nodes interleaved in the same terminal. This is a much more efficient way to manage and run a multi-node application.

---

## Practical Exercise

**Objective:** Modify the launch file to change the behavior of your nodes.

**Instructions:**
1.  Open your `talker_listener_launch.py` file.
2.  **Remapping a Topic:** Let's say you want your nodes to communicate on a different topic named `/my_chatter` instead of `/chatter`. You can add a `remappings` argument to the `Node` action.
3.  Modify both the `talker` and `listener` Node definitions to include the remapping.

**Modified Code Snippet:**
```python
# ... (inside generate_launch_description)
    return LaunchDescription([
        Node(
            package='my_python_pkg',
            executable='talker',
            name='my_talker_node',
            remappings=[
                ('chatter', 'my_chatter') # Remap the 'chatter' topic to 'my_chatter'
            ]
        ),
        Node(
            package='my_python_pkg',
            executable='listener',
            name='my_listener_node',
            remappings=[
                ('chatter', 'my_chatter')
            ]
        ),
    ])
```

4.  Save the file and run the launch file again: `ros2 launch my_python_pkg talker_listener_launch.py`. The nodes should still communicate correctly.
5.  **Verification:** In a new terminal, run `ros2 topic list`. You should now see `/my_chatter` in the list instead of `/chatter`, confirming that your remapping was successful.

---

## Summary

-   ROS 2 code is organized into **packages**, which contain source code, a `package.xml` manifest, and a `setup.py` build script.
-   `package.xml` defines the package's metadata and its **dependencies** on other ROS 2 packages.
-   `setup.py` defines how to install the Python code and creates executables via the `entry_points` dictionary.
-   The `colcon build` command compiles the entire workspace, and `source install/setup.bash` makes the newly built packages available in your environment.
-   **Launch files** are powerful Python scripts used to start, configure, and manage multiple nodes, which is essential for running complex robotic applications.

---
## Resources
- [Creating a ROS 2 Package (Tutorial)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Creating a launch file (Tutorial)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Python Packages (`ament_python`) Documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)
- [Colcon Documentation](https://colcon.readthedocs.io/en/released/)