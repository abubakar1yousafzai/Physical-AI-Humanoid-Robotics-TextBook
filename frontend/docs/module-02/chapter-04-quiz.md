---
title: "Chapter 4: Quiz"
---
import Quiz from '@site/src/components/Quiz';

# Chapter 4: Launch Files and Parameters Quiz

<Quiz questions={[
  {
    id: 1,
    question: "What is the primary purpose of a ROS 2 launch file?",
    options: [
      { id: 'A', text: "To define custom message types." },
      { id: 'B', text: "To build and compile a ROS 2 workspace." },
      { id: 'C', text: "To start, stop, and configure multiple nodes with a single command." },
      { id: 'D', text: "To write unit tests for a ROS 2 node." }
    ],
    correctAnswer: 'C',
    explanation: "Launch files are the standard way to manage the execution of a complex robotics application composed of many different nodes."
  },
  {
    id: 2,
    question: "In a Python launch file, what is the name of the main function that the launch system looks for?",
    options: [
      { id: 'A', text: "main()" },
      { id: 'B', text: "run_nodes()" },
      { id: 'C', text: "generate_launch_description()" },
      { id: 'D', text: "start_ros_application()" }
    ],
    correctAnswer: 'C',
    explanation: "This function is the designated entry point and must return a `LaunchDescription` object containing the list of actions to be executed."
  },
  {
    id: 3,
    question: "How do you make a launch file configurable from the command line?",
    options: [
      { id: 'A', text: "By using `DeclareLaunchArgument` to define an argument and `LaunchConfiguration` to use its value." },
      { id: 'B', text: "By hard-coding all the values directly in the `Node` actions." },
      { id: 'C', text: "By reading from a global environment variable inside the launch file." },
      { id: 'D', text: "By using a special `config` tag in the `package.xml` file." }
    ],
    correctAnswer: 'A',
    explanation: "This is the standard mechanism for creating reusable launch files where settings can be provided at runtime."
  },
  {
    id: 4,
    question: "What is the main benefit of using ROS 2 Parameters in a node?",
    options: [
      { id: 'A', text: "They make the node run faster." },
      { id: 'B', text: "They decouple the node's configuration from its source code, making it more flexible." },
      { id: 'C', text: "They automatically save all node data to a file." },
      { id: 'D', text: "They encrypt the communication between nodes." }
    ],
    correctAnswer: 'B',
    explanation: "Instead of hard-coding a value, you make it a parameter, which can then be easily changed via a launch file or command-line tool without recompiling the code."
  },
  {
    id: 5,
    question: "In a Python node, which function do you call to define a new parameter with a default value?",
    options: [
      { id: 'A', text: "self.create_parameter()" },
      { id: 'B', text: "self.get_parameter()" },
      { id: 'C', text: "self.set_parameter()" },
      { id: 'D', text: "self.declare_parameter()" }
    ],
    correctAnswer: 'D',
    explanation: "This function registers the parameter with the node and the ROS 2 Parameter Server, making it discoverable and configurable from the outside."
  },
  {
    id: 6,
    question: "Which command would you use to change the value of a parameter named `speed` on a running node named `/robot_driver` to `1.5`?",
    options: [
      { id: 'A', text: "ros2 param get /robot_driver speed 1.5" },
      { id: 'B', text: "ros2 param set /robot_driver speed 1.5" },
      { id: 'C', text: "ros2 run /robot_driver speed 1.5" },
      { id: 'D', text: "ros2 topic pub /robot_driver/speed std_msgs/Float64 \"{data: 1.5}\"" }
    ],
    correctAnswer: 'B',
    explanation: "The `ros2 param set` command allows you to dynamically change a parameter's value on a running node, which is extremely useful for live tuning and debugging."
  },
  {
    id: 7,
    question: "How are parameters typically passed to a node from a launch file?",
    options: [
      { id: 'A', text: "Using the `remappings` argument in the `Node` action." },
      { id: 'B', text: "By setting an environment variable before launching." },
      { id: 'C', text: "Using the `parameters` argument in the `Node` action, often with a list of dictionaries." },
      { id: 'D', text: "The node automatically reads them from a file named `params.yaml`." }
    ],
    correctAnswer: 'C',
    explanation: "This argument takes a list, allowing you to set multiple parameters for the node as it starts up, for example: `parameters=[{'param_name': 'param_value'}]`."
  }
]} />