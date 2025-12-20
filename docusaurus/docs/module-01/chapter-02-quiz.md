---
title: "Chapter 2: Quiz"
---
import Quiz from '@site/src/components/Quiz';

# Chapter 2: From Digital AI to Robots that Understand Physical Laws Quiz

<Quiz questions={[
  {
    id: 1,
    question: "What is the 'symbol grounding problem' in AI?",
    options: [
      { id: 'A', text: "A problem with using the wrong icons in a user interface." },
      { id: 'B', text: "The difficulty of linking abstract symbols in an AI's system to real-world meaning." },
      { id: 'C', text: "An error in a robot's electrical grounding." },
      { id: 'D', text: "A challenge in teaching AI to write symbolic poetry." }
    ],
    correctAnswer: 'B',
    explanation: "The difficulty of linking abstract symbols in an AI's system to real-world meaning. This is a core philosophical and technical challenge that Physical AI addresses by connecting symbols to sensory data and actions."
  },
  {
    id: 2,
    question: "Which of these is NOT a primary benefit of using simulation for robot training?",
    options: [
      { id: 'A', text: "It guarantees that the learned policy will work perfectly in the real world." },
      { id: 'B', text: "It is generally safer than training with a real, physical robot." },
      { id: 'C', text: "It allows training to happen faster than real-time." },
      { id: 'D', text: "It allows for massive parallelization of training scenarios." }
    ],
    correctAnswer: 'A',
    explanation: "It guarantees that the learned policy will work perfectly in the real world. Due to the sim-to-real gap, a perfect guarantee is not possible, which is why techniques to bridge this gap are so important."
  },
  {
    id: 3,
    question: "What is 'domain randomization'?",
    options: [
      { id: 'A', text: "Choosing a random domain name for a robot's control server." },
      { id: 'B', text: "Training an AI in many simulations with varying physical and visual properties." },
      { "id": 'C', text: "A cybersecurity technique to randomize network ports." },
      { id: 'D', text: "Letting a robot move randomly to explore its environment." }
    ],
    correctAnswer: 'B',
    explanation: "Training an AI in many simulations with varying physical and visual properties. This forces the AI to learn a more robust policy that is not over-fitted to a single, perfect simulation, helping it to cross the sim-to-real gap."
  },
  {
    id: 4,
    question: "In the PyBullet code, what does `p.stepSimulation()` do?",
    options: [
      { id: 'A', text: "It loads a new robot model." },
      { id: 'B', text: "It resets the entire simulation." },
      { id: 'C', text: "It advances the physics simulation forward by a single discrete time step." },
      { id: 'D', text: "It captures a screenshot of the current view." }
    ],
    correctAnswer: 'C',
    explanation: "It advances the physics simulation forward by a single discrete time step. The simulation loop repeatedly calls this function to make time progress in the virtual world and update the state of all objects according to the physics engine."
  },
  {
    id: 5,
    question: "What is the most likely reason a policy learned in a 'clean' simulation would fail in the real world?",
    options: [
      { id: 'A', text: "The real world has different programming languages." },
      { id: 'B', text: "The simulation was running too fast." },
      { id: 'C', text: "The real world has unmodeled effects like sensor noise and friction." },
      { id: 'D', text: "The computer running the simulation was not powerful enough." }
    ],
    correctAnswer: 'C',
    explanation: "The real world has unmodeled effects like sensor noise and friction. These small, unpredictable variations are a primary cause of the sim-to-real gap, as the AI was not exposed to them during its training."
  },
  {
    id: 6,
    question: "What does the `restitution` parameter, modified in the practical exercise, control?",
    options: [
      { id: 'A', text: "The object's mass." },
      { id: 'B', text: "The object's color." },
      { id: 'C', text: "The object's bounciness." },
      { id: 'D', text: "The force of gravity." }
    ],
    correctAnswer: 'C',
    explanation: "The object's bounciness. Restitution is a physics term for the coefficient that determines how much kinetic energy is retained after a collision. A value of 1.0 would be a perfectly elastic bounce."
  },
  {
    id: 7,
    question: "Why would changing an object's `baseMass` in PyBullet NOT change how fast it falls in a vacuum?",
    options: [
      { id: 'A', text: "PyBullet's gravity simulation is inaccurate." },
      { id: 'B', text: "Mass only affects horizontal movement, not vertical." },
      { id: 'C', text: "Acceleration due to gravity is independent of an object's mass." },
      { id: 'D', text: "The `stepSimulation` function automatically compensates for mass." }
    ],
    correctAnswer: 'C',
    explanation: "Acceleration due to gravity is independent of an object's mass. This is a fundamental principle of physics (first famously demonstrated by Galileo), and physics simulators like PyBullet correctly model this behavior."
  }
]} />