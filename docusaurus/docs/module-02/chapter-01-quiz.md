---
title: "Chapter 1: Quiz"
---
import Quiz from '@site/src/components/Quiz';

# Chapter 1: ROS 2 Architecture and Core Concepts Quiz

<Quiz questions={[
  {
    id: 1,
    question: "What is the primary function of ROS 2?",
    options: [
      { id: 'A', text: "To serve as a traditional operating system like Linux." },
      { id: 'B', text: "To provide a hardware abstraction layer for specific robot models." },
      { id: 'C', text: "To provide a standardized middleware and framework for robot software development." },
      { id: 'D', text: "To be a physics simulator for testing robot algorithms." }
    ],
    correctAnswer: 'C',
    explanation: "ROS 2 is not an OS itself but a framework that runs on top of an OS (like Linux or Windows) to help developers build modular and reusable robotics applications."
  },
  {
    id: 2,
    question: "In the ROS 2 graph, what is a 'Node'?",
    options: [
      { id: 'A', text: "A message being sent between processes." },
      { id: 'B', text: "A physical connection between two robot parts." },
      { id: 'C', text: "A process that performs computation (e.g., a sensor driver or a path planner)." },
      { id: 'D', text: "A specific type of sensor, like a camera." }
    ],
    correctAnswer: 'C',
    explanation: "A key design principle of ROS is to break down a complex system into many small, single-purpose nodes that communicate with each other, which enhances modularity and fault isolation."
  },
  {
    id: 3,
    question: "You need to continuously stream camera images from a camera node to a separate image processing node. Which ROS 2 communication mechanism is most appropriate?",
    options: [
      { id: 'A', text: "Topics (Publish/Subscribe)" },
      { id: 'B', text: "Services (Request/Response)" },
      { id: 'C', text: "Actions" },
      { id: 'D', text: "Direct TCP Sockets" }
    ],
    correctAnswer: 'A',
    explanation: "Topics are designed for continuous, one-to-many data streams where the publisher does not need a direct reply. This is a perfect fit for sensor data like camera images."
  },
  {
    id: 4,
    question: "When would you choose to use a ROS 2 Service instead of a Topic?",
    options: [
      { id: 'A', text: "When you need to send data to multiple listeners at once." },
      { id: 'B', text: "When the data stream is continuous and high-frequency." },
      { id: 'C', text: "When you need to trigger a remote computation and wait for a direct, guaranteed response." },
      { id: 'D', text: "When the task takes a very long time to complete and you need feedback." }
    ],
    correctAnswer: 'C',
    explanation: "Services are designed for synchronous, request-response transactions, much like a remote function call, which is ideal for tasks that should complete quickly and return a result."
  },
  {
    id: 5,
    question: "What is the main advantage of an Action over a Service?",
    options: [
      { id: 'A', text: "Actions are simpler to implement." },
      { id: 'B', text: "Actions are faster for quick requests." },
      { id: 'C', text: "Actions can handle long-running tasks, provide continuous feedback, and can be cancelled." },
      { id: 'D', text: "Actions can communicate with multiple servers at once." }
    ],
    correctAnswer: 'C',
    explanation: "Actions can handle long-running tasks, provide continuous feedback, and can be cancelled. This makes them suitable for goal-oriented behaviors like 'navigate to a point,' where the client needs updates on progress and the ability to abort the mission."
  },
  {
    id: 6,
    question: "Which command-line tool would you use to view the data being published on a specific topic in real-time?",
    options: [
      { id: 'A', text: "ros2 topic list" },
      { id: 'B', text: "ros2 node info" },
      { id: 'C', text: "ros2 topic echo <topic_name>" },
      { id: 'D', text: "rqt_graph" }
    ],
    correctAnswer: 'C',
    explanation: "This command subscribes to the given topic and prints the content of every message it receives directly to the terminal, making it an indispensable tool for debugging."
  },
  {
    id: 7,
    question: "What does the number `10` in `self.create_publisher(String, 'hello_world', 10)` signify?",
    options: [
      { id: 'A', text: "The publisher will only send 10 messages." },
      { id: 'B', text: "The message data is limited to 10 characters." },
      { id: 'C', text: "The topic name can be at most 10 characters long." },
      { id: 'D', text: "It is the Quality of Service (QoS) queue size, which buffers messages if the subscriber is not ready." }
    ],
    correctAnswer: 'D',
    explanation: "It is the Quality of Service (QoS) queue size. This setting determines how many outgoing messages are buffered. If messages are published faster than they can be sent over the network, this queue will hold the most recent ones up to its size limit."
  }
]} />
