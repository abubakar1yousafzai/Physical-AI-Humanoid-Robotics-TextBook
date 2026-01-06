---
title: "Chapter 1: ROS 2 Architecture and Core Concepts"
---

# Chapter 1: ROS 2 Architecture and Core Concepts

-   **Learning Objectives**:
    -   Describe the purpose and benefits of using a robotics framework like ROS 2.
    -   Identify and define the core components of the ROS 2 graph: Nodes, Topics, Services, and Actions.
    -   Explain the publish-subscribe communication model and its role in ROS 2.
    -   Differentiate between the use cases for topics, services, and actions.
    -   Use basic ROS 2 command-line tools to inspect a running system.

## Introduction 

Imagine building a complex robot from scratch. You would need to write code to read data from a dozen different sensors, write drivers to control dozens of motors, implement algorithms for navigation and decision-making, and create a way for all these disparate pieces of software to communicate with each other reliably and in real-time. This is an monumental task, and for a long time, it was a major barrier to entry in robotics. Every research lab and company had to reinvent the wheel, creating their own proprietary software plumbing.

This is the problem that the Robot Operating System (ROS) was created to solve. ROS is not a traditional operating system like Windows or Linux. Rather, it is a **middleware** or a **software framework** that provides a standardized structure for robot software. It provides libraries, tools, and conventions that simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS 2 is the second generation of this framework, redesigned from the ground up to be more robust, secure, and suitable for commercial and mission-critical applications.

In this chapter, we will dissect the fundamental architecture of ROS 2. We will learn about the "graph" of communication that forms the backbone of any ROS 2 system. You will be introduced to the four most important concepts in ROS 2: **Nodes**, which are the processes that perform computation; **Topics**, which provide a continuous stream of data; **Services**, which are used for request-response interactions; and **Actions**, which are for long-running, feedback-driven tasks. Understanding this architecture is the first and most crucial step to becoming a proficient ROS 2 developer.

---

## Main Content 

### Why a Robotics Framework? The Philosophy of ROS

Before diving into the technical details, it's important to understand the philosophy behind ROS. ROS was designed to encourage collaborative robotics software development. It achieves this through several key principles:

1.  **Peer-to-Peer Communication:** ROS systems are a distributed network of processes. This allows for a modular design where different components can be developed independently and run on different hardware, as long as they are connected to the same network.
2.  **Language-Agnostic:** While the main supported languages are Python and C++, ROS 2's communication system is language-independent. A node written in Python can seamlessly communicate with a node written in C++.
3.  **Thin is Better:** The ROS framework itself is designed to be as thin as possible. It doesn't wrap your main() function. Instead, it provides libraries that you can use in your own applications, making it easy to integrate ROS with existing codebases.
4.  **Rich Tooling:** A major strength of ROS is its vast ecosystem of tools for visualization (RViz2), simulation (Gazebo), debugging, and data logging (ros2 bag).

### The ROS 2 Graph: A Network of Nodes

The fundamental concept in ROS 2 is the **ROS 2 graph**. This is a network of ROS 2 processes (nodes) that are communicating with each other. It's called a "graph" because we can visualize the nodes as vertices and the communication between them as edges.

:::tip[Visualizing the Graph]
A powerful command-line tool, `rqt_graph`, allows you to see the ROS 2 graph of a running system in real-time. It's an invaluable tool for debugging and understanding how a complex system is structured.
:::

The primary components of the graph are:

-   **Nodes:** A node is a process that performs computation. A well-designed robot software system is composed of many nodes, each with a single, well-defined purpose. For example, you might have one node for controlling the wheel motors, another node for reading from a laser scanner, and a third node for planning a path. This modularity makes the system easier to debug, test, and reuse.
-   **Topics (Publish/Subscribe):** The most common way for nodes to communicate. Topics are named buses over which nodes exchange messages. A node can **publish** data to a topic (e.g., a camera node publishing a stream of images), and any number of other nodes can **subscribe** to that topic to receive the data. This is a one-to-many communication model. The publisher doesn't know or care who is subscribed; it just sends the data. This decoupling is a core strength of ROS.
-   **Services (Request/Response):** Services are used for two-way communication where a response is expected. It's a request-response model, similar to a function call. One node acts as a **service server**, advertising a service. Another node acts as a **service client**, calling the service with a request and waiting for a response. This is a one-to-one communication model and is synchronous (the client blocks until the response is received). An example would be a service to "capture a single high-resolution image."
-   **Actions (Long-Running Tasks):** Actions are for long-running, asynchronous tasks that provide feedback during their execution. They are similar to services but are designed for tasks that might take a significant amount of time, like "navigate to a waypoint." An **action server** provides the functionality, and an **action client** sends a goal. The key difference from services is that the client can receive continuous feedback (e.g., the robot's current distance to the goal) and can also cancel the goal mid-execution.

| Communication Type | Use Case | Analogy | Type | Synchronicity |
| :--- | :--- | :--- | :--- | :--- |
| **Topics** | Continuous data streams (e.g., sensor data, robot state). | A radio station broadcasting news. Anyone can tune in. | One-to-Many | Asynchronous |
| **Services** | Quick, remote procedure calls where an immediate answer is needed. | Making a phone call and asking a direct question. | One-to-One | Synchronous |
| **Actions** | Long-running tasks where feedback and preemption are required. | Ordering a pizza for delivery. You get updates (being made, out for delivery) and can cancel. | One-to-One | Asynchronous |

### Deeper Dive: ROS 2 Messages

Nodes communicate by passing ROS **messages** on topics, services, and actions. A message is simply a data structure with a defined type. ROS 2 provides a rich set of standard message types (e.g., `sensor_msgs/msg/Image`, `geometry_msgs/msg/Twist`), and you can also define your own custom message types.

For example, a common message for sending velocity commands to a mobile robot is `geometry_msgs/msg/Twist`. It looks like this:
```
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
```
And a `Vector3` is simply:
```
float64 x
float64 y
float64 z
```
So, to make a robot move forward, a navigation node would publish a `Twist` message on a topic (often named `/cmd_vel`) with `linear.x` set to a positive value. The motor controller node, which subscribes to this topic, would receive the message and translate it into the appropriate wheel speeds.

### Command-Line Tools for Inspection

ROS 2 comes with a powerful set of command-line tools that allow you to introspect, debug, and interact with a running system. The main entry point is the `ros2` command.

**Installation (on Ubuntu 22.04 with ROS 2 Humble):**
Follow the official ROS 2 installation guide. A key part of using ROS is "sourcing" the setup file in your terminal to make the commands available:
```bash
# Add this to your ~/.bashrc file to source it automatically in new terminals
source /opt/ros/humble/setup.bash
```

**Common Commands:**
-   **`ros2 topic list`**: Lists all the active topics in the system.
-   **`ros2 topic echo <topic_name>`**: Prints the data being published on a specific topic to the screen. Incredibly useful for debugging.
-   **`ros2 node list`**: Lists all the running nodes.
-   **`ros2 service list`**: Lists all the available services.
-   **`ros2 service call <service_name> <service_type> '<request_in_yaml>'`**: Calls a service from the command line.
-   **`rqt_graph`**: Launches a GUI that visualizes the graph of nodes and topics.

---

## Code Example: A Simple Publisher in Python

This example demonstrates how to create a minimal ROS 2 node in Python that continuously publishes a "Hello World" message to a topic.

**Prerequisites:** A working ROS 2 Humble installation.

**Instructions:**
1.  Create a ROS 2 package (we will cover this in detail in a later chapter, but for now, you can save the script in a simple file).
2.  Save the following code as `hello_publisher.py`.

**Code:**
```python
# Import the necessary ROS 2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # We will use the standard String message type

class HelloWorldPublisher(Node):
    """
    A simple ROS 2 node that publishes 'hello world' messages.
    """
    def __init__(self):
        # Initialize the Node with the name 'hello_world_publisher'
        super().__init__('hello_world_publisher')
        
        # Create a publisher. 
        # It will publish messages of type String to the 'hello_world' topic.
        # The '10' is the queue size - a quality of service (QoS) setting.
        self.publisher_ = self.create_publisher(String, 'hello_world', 10)
        
        # Create a timer that will call the timer_callback function every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # A counter for our messages
        self.i = 0

    def timer_callback(self):
        # This function is called by the timer
        msg = String()
        msg.data = f'Hello World: {self.i}'
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Increment the counter
        self.i += 1

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create an instance of our node
    hello_world_publisher = HelloWorldPublisher()
    
    # "Spin" the node, which allows it to process callbacks (like the timer)
    # This will block until the node is shut down (e.g., by Ctrl+C)
    rclpy.spin(hello_world_publisher)
    
    # Clean up and destroy the node
    hello_world_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Practical Exercise

**Objective:** To use ROS 2 command-line tools to interact with the "Hello World" publisher node.

**Instructions:**
1.  Open a new terminal. Make sure you have sourced your ROS 2 setup file (`source /opt/ros/humble/setup.bash`).
2.  Run the Python publisher node you just created:
    ```bash
    python3 hello_publisher.py
    ```
    You should see the "Publishing: ..." messages appearing in this terminal.
3.  Open a **second terminal**. Source the setup file again.
4.  **Inspect the system:**
    -   Run `ros2 node list`. You should see `/hello_world_publisher` in the list.
    -   Run `ros2 topic list`. You should see `/hello_world` in the list.
5.  **Listen to the topic:**
    -   Run `ros2 topic echo /hello_world`.
    -   You should now see the "Hello World: X" messages appearing in your second terminal. This demonstrates that the message is being broadcast over the ROS 2 network.
6.  **Visualize the graph:**
    -   In a third terminal (optional, requires GUI), run `rqt_graph`. A window should appear showing one node (`/hello_world_publisher`) publishing to one topic (`/hello_world`).
7.  Close all programs and terminals when you are finished.

---

## Summary

-   **ROS 2** is a middleware framework that provides a standardized, modular, and language-agnostic way to build complex robot software.
-   The **ROS 2 Graph** consists of **Nodes** (processes) that communicate with each other using three primary mechanisms.
-   **Topics** use a publish-subscribe model for continuous, one-to-many data streams (e.g., sensor data).
-   **Services** use a request-response model for synchronous, one-to-one communication where an immediate answer is needed.
-   **Actions** are used for long-running, asynchronous tasks that provide feedback and can be cancelled.
-   Command-line tools like `ros2 topic echo` and `rqt_graph` are essential for debugging and understanding running ROS 2 systems.

---
## Resources
- [ROS 2 Documentation - Core Concepts](https://docs.ros.org/en/humble/Concepts.html)
- [Understanding ROS 2 Nodes (Tutorial)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [Understanding ROS 2 Topics (Tutorial)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [RCLPY (Python Client Library) API Documentation](https://docs.ros2.org/latest/api/rclpy/api.html)