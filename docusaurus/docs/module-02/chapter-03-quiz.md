---
title: "Chapter 3: Quiz"
---
import Quiz from '@site/src/components/Quiz';

# Chapter 3: Building ROS 2 Packages with Python Quiz

<Quiz questions={[
  {
    id: 1,
    question: "What is the primary role of the `package.xml` file?",
    options: [
      { id: 'A', text: "To contain the main Python source code for the node." },
      { id: 'B', text: "To define the package's metadata, such as its name, version, and dependencies." },
      { id: 'C', text: "To configure the visual appearance of the robot model in a simulator." },
      { id: 'D', text: "To list the sequence of nodes to run when the package is launched." }
    ],
    correctAnswer: 'B',
    explanation: "The `package.xml` file is the manifest that describes the package to the ROS 2 ecosystem, including what other packages it needs to build and run correctly."
  },
  {
    id: 2,
    question: "In `setup.py`, what is the purpose of the `entry_points` dictionary?",
    options: [
      { id: 'A', text: "It specifies the physical ports on the robot for sensor connections." },
      { id: 'B', text: "It lists all the functions in the Python scripts that can be called externally." },
      { id: 'C', text: "It creates command-line executables by mapping a name to a `main` function in a Python script." },
      { id: 'D', text: "It defines the starting position of the robot in a simulation." }
    ],
    correctAnswer: 'C',
    explanation: "This is the standard `setuptools` mechanism that ROS 2 uses to allow you to run your nodes with commands like `ros2 run <package_name> <executable_name>`."
  },
  {
    id: 3,
    question: "What does the `colcon build` command do in a ROS 2 workspace?",
    options: [
      { id: 'A', text: "It only cleans the workspace by deleting temporary files." },
      { id: 'B', text: "It finds all packages in the `src` directory, compiles them, and places the results in the `install` and `build` directories." },
      { id: 'C', text: "It launches all the nodes defined in the workspace's launch files." },
      { id: 'D', text: "It downloads and installs ROS 2 from the internet." }
    ],
    correctAnswer: 'B',
    explanation: "It is the primary tool for building an entire ROS 2 workspace."
  },
  {
    id: 4,
    question: "Why must you run `source install/setup.bash` after building a workspace?",
    options: [
      { id: 'A', text: "To delete the source files and save space." },
      { id: 'B', text: "To update the current terminal's environment variables so it can find your new packages and executables." },
      { id: 'C', text: "To automatically start all the nodes you just built." },
      { id: 'D', text: "To convert your Python code to C++ for better performance." }
    ],
    correctAnswer: 'B',
    explanation: "This 'sourcing' action adds your workspace's `install` directory to the system's path, making your custom packages and nodes discoverable by ROS 2 tools."
  },
  {
    id: 5,
    question: "What is the main advantage of using a ROS 2 launch file?",
    options: [
      { id: 'A', text: "It automatically writes the Python code for your nodes." },
      { id: 'B', text: "It allows you to start, stop, and configure multiple nodes with a single command." },
      { id: 'C', text: "It guarantees that your code will have no bugs." },
      { id: 'D', text: "It makes your Python nodes run as fast as C++ nodes." }
    ],
    correctAnswer: 'B',
    explanation: "This is essential for managing real-world robotics applications, which are often composed of many interconnected nodes."
  },
  {
    id: 6,
    question: "In a Python launch file, what does the `Node` action from `launch_ros.actions` do?",
    options: [
      { id: 'A', text: "It defines a new ROS 2 message type." },
      { id: 'B', text: "It describes how to start a single ROS 2 node, specifying its package and executable." },
      { id: 'C', text: "It creates a physical connection in the ROS 2 graph visualization." },
      { id: 'D', text: "It checks the health and status of a running node." }
    ],
    correctAnswer: 'B',
    explanation: "The launch system processes this action by executing the specified `ros2 run` command with the given parameters."
  },
  {
    id: 7,
    question: "If you use the `remappings` argument in a launch file, what are you doing?",
    options: [
      { id: 'A', text: "Changing the name of the node itself." },
      { id: 'B', text: "Assigning a new IP address to the node." },
      { id: 'C', text: "Changing the name of a topic, service, or action that the node uses." },
      { id: 'D', text: "Increasing the processing priority of the node." }
    ],
    correctAnswer: 'C',
    explanation: "Remapping is a powerful feature that allows you to reconnect the communication pathways of your nodes at runtime without changing their source code."
  }
]} />
