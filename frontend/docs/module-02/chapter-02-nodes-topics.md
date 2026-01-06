---
title: "Chapter 2: Nodes, Topics, Services, and Actions in Practice"
---

# Chapter 2: Nodes, Topics, Services, and Actions in Practice


-   **Learning Objectives**:
    -   Write a basic ROS 2 publisher and subscriber node in Python.
    -   Understand the structure of a ROS 2 Python package.
    -   Use `colcon` to build and run your ROS 2 package.
    -   Implement a simple Service and Action client/server pair.
    -   Use command-line tools to interact with your custom nodes, services, and actions.

## Introduction 

In the previous chapter, we introduced the theoretical architecture of the ROS 2 graph, composed of nodes, topics, services, and actions. Now, it's time to put that theory into practice. This chapter is a hands-on tutorial where you will write, build, and run your own ROS 2 nodes in Python. We will demystify the process of creating a ROS 2 application from scratch, giving you the core skills needed to develop any robotics application.

We will begin by building the most common and fundamental components: a **publisher** node that sends out data and a **subscriber** node that receives it. This will solidify your understanding of the publish-subscribe model, the workhorse of ROS 2 communication. You will learn how to define message types and see data flowing between your two running processes.

Next, we will move beyond simple data streaming to explore the request-response paradigm by implementing a **service**. You'll create a server node that can perform a calculation on demand and a client node that requests the calculation and waits for the result. Finally, we will tackle **actions**, the mechanism for long-running tasks. You will build an action server that simulates a time-consuming process and an action client that can monitor its progress and receive the final outcome. Along the way, you'll learn how to organize your code into a proper ROS 2 package and use the standard `colcon` build tool. This chapter is all about writing code and seeing it run.

---

## Main Content 

### Setting Up Your ROS 2 Workspace

Before we start writing code, we need a place to put it. In ROS 2, code is organized into **packages**, and packages are stored in a **workspace**.

A workspace is simply a directory containing your ROS 2 packages. Let's create one.

**Instructions:**
1. Open a new terminal. Remember to source your ROS 2 setup file (`source /opt/ros/humble/setup.bash`).
2. Create a directory for your workspace and a `src` folder inside it. The `src` folder is where you will put your package source code.
   ```bash
   mkdir -p ros2_ws/src
   cd ros2_ws
   ```
This `ros2_ws` directory is now your workspace.

### Creating a ROS 2 Package in Python

A ROS 2 package is a directory with a specific structure and a `package.xml` file that contains metadata about the package (like its name, version, and dependencies). We can use the `ros2 pkg create` command to generate a boilerplate package for us.

**Instructions:**
1. Navigate into the `src` directory of your workspace:
   ```bash
   cd src
   ```
2. Run the package creation command:
   ```bash
   ros2 pkg create --build-type ament_python --node-name my_first_node my_python_pkg
   ```
Let's break down this command:
- `--build-type ament_python`: Specifies that we are creating a Python package. `ament` is the build system used by ROS 2.
- `--node-name my_first_node`: This is a convenience that also creates a basic executable Python script with this name.
- `my_python_pkg`: This is the name of our new package.

You should now have a new directory called `my_python_pkg` with the following structure:
```
my_python_pkg/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── my_python_pkg
└── my_python_pkg/
    ├── __init__.py
    └── my_first_node.py
```

### Implementing a Publisher and Subscriber

Now let's modify the generated files to create a publisher and a subscriber. We will create two separate nodes within our package.

**1. The Talker (Publisher) Node**

This node will publish a string message to a topic every second.

-   **File:** `my_python_pkg/my_python_pkg/talker.py` (You can rename `my_first_node.py` or create a new file)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.get_logger().info('Talker node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**2. The Listener (Subscriber) Node**

This node will subscribe to the `chatter` topic and print any messages it receives.

-   **File:** `my_python_pkg/my_python_pkg/listener.py` (Create this new file)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Listener node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**3. Updating `setup.py`**

We need to tell the build system about our new executable scripts. Open `setup.py` and modify the `entry_points` section to look like this:

```python
# ... (other setup code)
'console_scripts': [
    'talker = my_python_pkg.talker:main',
    'listener = my_python_pkg.listener:main',
],
# ... (other setup code)
```
This tells ROS 2 that we want to create two executable commands, `talker` and `listener`, and that they should run the `main` function from our respective Python scripts.

### Building and Running with `colcon`

`colcon` is the standard build tool for ROS 2. It finds all the packages in your workspace, builds them, and generates setup files so you can run your code.

**Instructions:**
1.  Navigate to the root of your workspace (`cd ~/ros2_ws`).
2.  Run the build command:
    ```bash
    colcon build
    ```
3.  After the build finishes, you need to source the new setup file that `colcon` created. This will add your new package and executables to your environment.
    ```bash
    source install/setup.bash
    ```
4.  Now, run the nodes!
    -   In one terminal, run the talker:
        ```bash
        ros2 run my_python_pkg talker
        ```
    -   In a second terminal, run the listener:
        ```bash
        ros2 run my_python_pkg listener
        ```
You should see the talker publishing messages and the listener receiving them! You have successfully created a distributed robotics application.

### Implementing a Service

Let's add a service to our package. The service will take two integers and return their sum.

**1. Define a Custom Service Type**

First, we need to define the data structure for our service's request and response.
-   Create a new directory: `my_python_pkg/srv`
-   Create a new file inside it: `my_python_pkg/srv/AddTwoInts.srv`

```
int64 a
int64 b
---
int64 sum
```
The `---` separates the request part (above) from the response part (below).

**2. Update Package Configuration**
-   In `package.xml`, add these lines to tell ROS 2 we need to build messages:
    ```xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
-   In `CMakeLists.txt` (yes, even for Python packages for this part), add:
    ```cmake
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME} "srv/AddTwoInts.srv")
    ```

**3. Create the Server and Client Nodes**
(The full code is omitted for brevity, but it would be very similar in structure to the publisher/subscriber, using `create_service` and `create_client` respectively).

### Implementing an Action

Finally, let's consider an action. Imagine we want to tell the robot to "rotate by N degrees." This is a long-running task where we want feedback.

**1. Define a Custom Action Type**
-   Create a new directory: `my_python_pkg/action`
-   Create a new file: `my_python_pkg/action/Rotate.action`

```
# Goal definition
float32 degrees
---
# Result definition
bool success
---
# Feedback definition
float32 degrees_rotated
```
The `---` separators divide the goal, result, and feedback parts of the action definition.

**2. Update Package Configuration**
Similar to services, you would need to update `package.xml` and `CMakeLists.txt` to generate the action interface files.

**3. Create the Action Server and Client Nodes**
(Again, the code is similar in structure, using `ActionServer` and `ActionClient` from the `rclpy.action` library).

---

## Practical Exercise

**Objective:** Use the command-line interface to call the "Add Two Ints" service (assuming you have built and are running the service server from the ROS 2 tutorials).

**Instructions:**
1.  Make sure a service server is running. You can run the example from the official ROS 2 tutorials:
    ```bash
    ros2 run examples_rclpy_minimal_service service
    ```
2.  In another terminal, list the available services to find the correct name:
    ```bash
    ros2 service list
    ```
    You should see `/add_two_ints`.
3.  Get the type of the service:
    ```bash
    ros2 service type /add_two_ints
    ```
    This should return `example_interfaces/srv/AddTwoInts`.
4.  Call the service from the command line, providing the request data in YAML format:
    ```bash
    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"
    ```
**Expected Output:**
The service client will wait for the response and then print it. You should see the `sum` field with the value `15`. This exercise demonstrates how to interact with and test services without needing to write a dedicated client node.

---
## Summary

-   ROS 2 code is organized into **packages** within a **workspace**.
-   The `ros2 pkg create` command is used to generate a new package from a template.
-   The `colcon build` command is used to build all packages in a workspace.
-   A **publisher** node uses `create_publisher()` to send messages on a topic.
-   A **subscriber** node uses `create_subscription()` to receive messages and process them in a callback function.
-   **Services** and **Actions** require custom interface definitions (`.srv` and `.action` files) and updates to the package's configuration files to be built correctly.
-   Command-line tools like `ros2 run` and `ros2 service call` are essential for running and debugging your nodes.

---
## Resources
- [Creating a ROS 2 Package (Tutorial)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Writing a Simple Service and Client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Writing a Simple Action Server and Client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Action-Service-And-Client.html)
- [`colcon` Build Tool Documentation](https://colcon.readthedocs.io/en/released/)