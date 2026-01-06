---
title: "Chapter 4: Launch Files and Parameters"
---

# Chapter 4: Launch Files and Parameters

-   **Learning Objectives**:
    -   Understand the role and power of ROS 2 launch files for managing complex applications.
    -   Create a Python-based launch file to start multiple nodes.
    -   Use launch arguments to make your launch files configurable and reusable.
    -   Implement ROS 2 parameters in a Python node to allow for runtime configuration.
    -   Modify a parameter's value from a launch file.

## Introduction 

As your robotics applications grow in complexity, manually running each node in a separate terminal becomes impractical, error-prone, and impossible to automate. The ROS 2 **Launch System** is the standard solution to this problem. It provides a powerful and flexible way to define, configure, and execute an entire multi-node application with a single command. A launch file is essentially a script that describes the "who, what, and how" of your application's startup process.

In this chapter, we will master the art of creating Python-based launch files. We will go beyond simply starting nodes and explore how to make our applications truly configurable. You will learn how to use **launch arguments** to pass settings into your launch file from the command line, making it reusable for different scenarios.

More importantly, we will dive into one of the most critical concepts for creating flexible robotics software: **ROS 2 Parameters**. Parameters are configuration values associated with a node that can be set at startup and, in some cases, changed during runtime. Instead of hard-coding a value like a robot's top speed or a sensor's topic name into your code, you declare it as a parameter. This decouples your code from its configuration, allowing you to tune your robot's behavior without ever needing to recompile your code. We will learn how to declare parameters in a Python node and how to set their values directly from a launch file.

---

## Main Content 

### Why We Need a Launch System

Consider a typical mobile robot. It might have:
-   A node for the wheel controllers.
-   A node for the LIDAR sensor.
-   A node for the IMU sensor.
-   A node for sensor fusion.
-   A SLAM node for mapping and localization.
-   A navigation node for path planning.

Starting all of these manually would be a nightmare. A launch file automates this, ensuring that all necessary nodes are started in a coordinated and configurable manner.

### Anatomy of a Python Launch File

A Python launch file is a Python script that contains a function called `generate_launch_description()`. This function must return a `LaunchDescription` object, which is a list of "actions" that the launch system will execute.

The most common action is `Node`, which we saw in the previous chapter. Let's look at a more advanced example.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_python_pkg',
            executable='talker',
            name='my_talker'
        ),
        Node(
            package='my_python_pkg',
            executable='listener',
            name='my_listener'
        )
    ])
```

To use this, you save it as a Python file (e.g., `my_launch_file.py`) inside a `launch` directory in your package and run it with:
`ros2 launch my_python_pkg my_launch_file.py`

### Making Launch Files Reusable with Arguments

Hard-coding values inside a launch file is not ideal. What if you want to run the same launch file but with a different configuration? **Launch Arguments** allow you to do this. You can declare an argument at the top of your launch file and then use its value when defining your nodes.

**Example: A configurable talker**

Let's imagine our talker node could be configured to say something other than "Hello World".

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare a launch argument
    my_message_arg = DeclareLaunchArgument(
        'my_message',
        default_value='Hello from the launch file!',
        description='The message to be published by the talker.'
    )

    # 2. Use the launch argument's value
    return LaunchDescription([
        my_message_arg, # Add the argument to the launch description
        
        Node(
            package='my_python_pkg',
            executable='talker',
            name='my_talker',
            # This is how we pass parameters to the node
            parameters=[{'message': LaunchConfiguration('my_message')}]
        )
    ])
```

-   `DeclareLaunchArgument`: This creates a new argument named `my_message` with a default value.
-   `LaunchConfiguration`: This object acts as a placeholder. When the launch file is executed, it will be replaced with the actual value of the `my_message` argument.
-   `parameters`: This is a special field in the `Node` action that is used to pass ROS 2 parameters to the node being launched.

Now you can run the launch file in two ways:
1.  **With the default value:**
    `ros2 launch my_python_pkg my_configurable_launch.py`
2.  **Overriding the value from the command line:**
    `ros2 launch my_python_pkg my_configurable_launch.py my_message:='A different message!'`

### Using Parameters in Your Python Node

For the launch file above to work, our `talker` node needs to be aware of the `message` parameter.

:::tip[What are Parameters?]
Parameters are configurable settings for a node. They are strongly typed (e.g., string, integer, float, boolean) and are stored on the **ROS 2 Parameter Server**, which is managed by the ROS 2 daemon. This allows other nodes and command-line tools to inspect and even change a node's parameters while it is running.
:::

Here is how you would modify the `talker.py` script to use a parameter:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.parameter import Parameter

class ParameterTalkerNode(Node):
    def __init__(self):
        super().__init__('parameter_talker')
        
        # 1. Declare the parameter and set its default value
        self.declare_parameter('message', 'Hello default world!')
        
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.get_logger().info('Parameter Talker node has been started.')

    def timer_callback(self):
        # 2. Get the current value of the parameter
        my_message = self.get_parameter('message').get_parameter_value().string_value

        msg = String()
        msg.data = f'{my_message}: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    # ... (main function is the same as before)
```
**How it Works:**
1.  `self.declare_parameter('message', 'Hello default world!')`: This is the crucial step. It tells the ROS 2 system that this node has a parameter named `message`. If no value is provided from the outside (e.g., from a launch file), it will use the default value `'Hello default world!'`.
2.  `self.get_parameter('message').get_parameter_value().string_value`: Inside the timer callback, we fetch the *current* value of the parameter. This means if the parameter is changed while the node is running, the node will start using the new value in its next callback.

By decoupling the message content from the code, we have made our node significantly more flexible and reusable.

### Interacting with Parameters from the Command Line

Just like topics and services, you can interact with a node's parameters using the `ros2 param` command.

**Instructions (assuming your `ParameterTalkerNode` is running):**
1.  **List parameters:**
    ```bash
    # List the parameters of a specific node
    ros2 param list /parameter_talker 
    ```
    You should see `message` in the output.
2.  **Get a parameter's value:**
    ```bash
    ros2 param get /parameter_talker message
    ```
    This will print the current string value of the parameter.
3.  **Set a parameter's value (dynamically):**
    ```bash
    ros2 param set /parameter_talker message "A new message from the CLI!"
    ```
    If you watch the output of your running talker node, you will see that it immediately starts publishing the new message! This demonstrates the power of dynamic reconfiguration in ROS 2.

---

## Practical Exercise

**Objective:** Create a launch file for a "safe" robot that uses parameters.

**Instructions:**
1.  Imagine you have a robot control node that has a parameter called `max_speed` (a float). The node will not allow the robot to move faster than this speed.
2.  Create a launch file named `safe_robot_launch.py`.
3.  Inside the launch file, declare a launch argument named `speed_limit` with a default value of `0.5` (m/s).
4.  Launch your robot control node (you can just use the `parameter_talker` executable as a stand-in for this exercise).
5.  In the `Node` action, pass the value of the `speed_limit` launch argument to the node's `max_speed` parameter.

**Challenge:** How would you run this launch file to set the robot's maximum speed to `1.2` m/s from the command line?

**Solution Snippet:**
```python
# safe_robot_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    speed_limit_arg = DeclareLaunchArgument(
        'speed_limit', default_value='0.5'
    )
    
    return LaunchDescription([
        speed_limit_arg,
        Node(
            package='my_python_pkg',
            executable='parameter_talker', # Using this as a placeholder
            name='robot_controller',
            parameters=[{'max_speed': LaunchConfiguration('speed_limit')}]
        )
    ])
```
**To run with the new speed limit:**
`ros2 launch my_python_pkg safe_robot_launch.py speed_limit:=1.2`

---

## Summary

-   **ROS 2 Launch Files** are Python scripts that provide a powerful, programmatic way to start and manage complex, multi-node applications.
-   **Launch Arguments** (`DeclareLaunchArgument`) make launch files configurable and reusable by allowing values to be passed in from the command line.
-   **ROS 2 Parameters** (`declare_parameter`) are settings associated with a node that allow its behavior to be configured at runtime, decoupling the code from the configuration.
-   You can set a node's parameters directly from a launch file using the `parameters` argument in the `Node` action.
-   The `ros2 param` command-line tool allows you to inspect and dynamically change a node's parameters while it is running.

---
## Resources
- [ROS 2 Launch System Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Using Parameters in a Class (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [ROS 2 Parameters (Concepts)](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html)
- [Launch File Best Practices](https://docs.ros.org/en/humble/Best-Practices/Launch-file-different-modes.html)