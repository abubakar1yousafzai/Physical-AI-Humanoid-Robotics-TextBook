---
title: "Chapter 4: Quiz"
---
import Quiz from '@site/src/components/Quiz';

# Chapter 4: Sensor Systems for Physical AI Quiz

<Quiz questions={[
  {
    id: 1,
    question: "What does SLAM stand for in the context of robotics?",
    options: [
      { id: 'A', text: "Systematic Localization and Mapping." },
      { id: 'B', text: "Simultaneous Localization and Mapping." },
      { id: 'C', text: "Sensor Logistics and Actuator Management." },
      { id: 'D', text: "Software Library for Autonomous Machines." }
    ],
    correctAnswer: 'B',
    explanation: "Simultaneous Localization and Mapping. This is a fundamental problem in robotics where a robot must build a map of its environment and track its own location within that map at the same time, often using data from sensors like LIDAR."
  },
  {
    id: 2,
    question: "What is the main advantage of a Depth Camera over a standard RGB camera?",
    options: [
      { id: 'A', text: "It has a higher resolution." },
      { id: 'B', text: "It is less expensive." },
      { id: 'C', text: "It provides per-pixel distance information, creating a 3D view." },
      { id: 'D', text: "It works better in very bright sunlight." }
    ],
    correctAnswer: 'C',
    explanation: "It provides per-pixel distance information, creating a 3D view. While an RGB camera only provides 2D color information, a depth camera adds the crucial third dimension, which is vital for tasks like obstacle avoidance and manipulation."
  },
  {
    id: 3,
    question: "An IMU typically combines which two primary sensors to estimate orientation?",
    options: [
      { id: 'A', text: "LIDAR and Camera." },
      { id: 'B', text: "Accelerometer and Gyroscope." },
      { id: 'C', text: "GPS and Magnetometer." },
      { id: 'D', text: "Force sensor and Torque sensor." }
    ],
    correctAnswer: 'B',
    explanation: "Accelerometer and Gyroscope. The accelerometer provides a long-term stable reference for gravity (down), while the gyroscope provides precise short-term measurement of rotation. Fusing them provides a stable orientation estimate."
  },
  {
    id: 4,
    question: "What is the primary purpose of applying a Gaussian blur before edge detection in the OpenCV example?",
    options: [
      { id: 'A', text: "To make the image brighter." },
      { id: 'B', text: "To convert the image to black and white." },
      { id: 'C', text: "To reduce image noise and prevent the detection of false edges." },
      { id: 'D', text: "To increase the sharpness of the edges." }
    ],
    correctAnswer: 'C',
    explanation: "To reduce image noise and prevent the detection of false edges. Edge detection algorithms are sensitive to noise. Blurring the image first smooths out minor noisy pixels, leading to cleaner and more meaningful edge detection results."
  },
  {
    id: 5,
    question: "What problem does 'Sensor Fusion' primarily aim to solve?",
    options: [
      { id: 'A', text: "The high cost of individual sensors." },
      { id: 'B', text: "The limited battery life available for sensors." },
      { id: 'C', text: "The individual weaknesses and noise inherent in any single sensor." },
      { id: 'D', text: "The difficulty in manufacturing new types of sensors." }
    ],
    correctAnswer: 'C',
    explanation: "The individual weaknesses and noise inherent in any single sensor. By intelligently combining data from multiple sensor types (e.g., a noisy but stable accelerometer with a precise but drifty gyroscope), a more accurate and reliable result can be achieved."
  },
  {
    id: 6,
    question: "Why would a 2D LIDAR be insufficient for a humanoid robot navigating a complex, multi-level environment?",
    options: [
      { id: 'A', text: "It cannot detect obstacles at all." },
      { id: 'B', text: "It only provides a single slice of the world and cannot see stairs, ceilings, or overhanging obstacles." },
      { id: 'C', text: "It is too large to fit on a humanoid robot." },
      { id: 'D', text: "It consumes too much power." }
    ],
    correctAnswer: 'B',
    explanation: "It only provides a single slice of the world and cannot see stairs, ceilings, or overhanging obstacles. A 2D LIDAR is great for flat environments, but for a humanoid to navigate a human world, it needs a full 3D understanding of its surroundings."
  },
  {
    id: 7,
    question: "What does a Kalman Filter often do in the context of sensor fusion?",
    options: [
      { id: 'A', text: "It compresses sensor data to save space." },
      { id: 'B', text: "It encrypts sensor data for security." },
      { id: 'C', text: "It provides an optimal estimate of a system's state by combining noisy measurements over time." },
      { id: 'D', text: "It converts sensor data from analog to digital format." }
    ],
    correctAnswer: 'C',
    explanation: "It provides an optimal estimate of a system's state by combining noisy measurements over time. It is a powerful mathematical tool used to recursively estimate the state of a dynamic system, making it ideal for fusing data from sensors like IMUs."
  }
]} />