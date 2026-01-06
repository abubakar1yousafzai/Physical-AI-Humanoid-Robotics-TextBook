---
id: chapter-04-quiz
title: "Quiz"
sidebar_label: "Chapter 4: Quiz"
---
import Quiz from '@site/src/components/Quiz';

# Chapter 4: Sim-to-Real Transfer Quiz

<Quiz questions={[
  {
    id: 1,
    question: "What is the 'reality gap' in the context of sim-to-real transfer?",
    options: [
      { id: "A", text: "The time delay between the simulation and the real world." },
      { id: "B", text: "The difference in graphical quality between the simulation and reality." },
      { id: "C", text: "The discrepancy between the simulation and the real world due to unmodeled effects and inaccurate parameters." },
      { id: "D", text: "The physical distance between the computer running the simulation and the robot." }
    ],
    correctAnswer: "C",
    explanation: "The 'reality gap' refers to the inevitable differences between a simulated environment and the real world. These discrepancies can arise from inaccurate physics modeling, sensor noise, and other unmodeled dynamics, making direct policy transfer challenging."
  },
  {
    id: 2,
    question: "What is the primary goal of Domain Randomization?",
    options: [
      { id: "A", text: "To make the simulation run as fast as possible." },
      { id: "B", text: "To make the simulation look as photorealistic as possible." },
      { id: "C", text: "To train a policy that is robust to variations between the simulation and the real world." },
      { id: "D", text: "To eliminate the need for a real-world robot." }
    ],
    correctAnswer: "C",
    explanation: "Domain randomization aims to bridge the reality gap by training the agent across a wide distribution of simulated environments. By exposing the policy to variations in physics and visuals, it learns to be more robust and generalize better to real-world conditions."
  },
  {
    id: 3,
    question: "Which of the following is NOT a parameter typically varied during domain randomization?",
    options: [
      { id: "A", text: "The reinforcement learning algorithm being used." },
      { id: "B", text: "The friction and mass of objects." },
      { id: "C", text: "The lighting conditions and textures in the environment." },
      { id: "D", text: "The position of the camera." }
    ],
    correctAnswer: "A",
    explanation: "Domain randomization focuses on varying the properties of the simulated environment (physics, visuals, task parameters), not the learning algorithm itself. The goal is to train a single policy that can function across many different environmental conditions."
  },
  {
    id: 4,
    question: "What is 'fine-tuning' in the context of sim-to-real transfer?",
    options: [
      { id: "A", text: "Adjusting the robot's hardware to match the simulation." },
      { id: "B", text: "Continuing the training of a simulation-trained policy with a small amount of real-world data." },
      { id: "C", text: "Manually adjusting the simulation parameters to perfectly match the real world." },
      { id: "D", text: "A method for directly transferring a policy without any changes." }
    ],
    correctAnswer: "B",
    explanation: "Fine-tuning is a common sim-to-real strategy where a policy is first pre-trained on a large amount of simulation data and then further trained on a small, precious amount of real-world data. This helps the policy adapt to the specific nuances of the real world."
  },
  {
    id: 5,
    question: "Which of the following is a known challenge that contributes to the reality gap?",
    options: [
      { id: "A", text: "Inaccurate physics modeling." },
      { id: "B", text: "Sensor noise and biases." },
      { id: "C", text: "Unmodeled dynamics like motor backlash." },
      { id: "D", text: "All of the above." }
    ],
    correctAnswer: "D",
    explanation: "The reality gap is a multi-faceted problem caused by many factors. Inaccuracies in physics simulation, differences in sensor behavior, and unmodeled real-world effects all contribute to the challenge of transferring policies from simulation to reality."
  },
  {
    id: 6,
    question: "Why is direct transfer of a policy from simulation to a real robot often unsuccessful?",
    options: [
      { id: "A", text: "Because the policy is not trained for long enough." },
      { id: "B", text: "Because the policy may have 'overfit' to the specific, clean dynamics of the simulation." },
      { id: "C", text: "Because real-world robots use a different programming language." },
      { id: "D", text: "Because the simulation is always slower than the real world." }
    ],
    correctAnswer: "B",
    explanation: "If a policy is trained in a single, non-randomized simulation, it may learn to exploit specific quirks of that environment. When transferred to the real world, which has different dynamics, the policy fails because it is not robust to these changes."
  },
  {
    id: 7,
    question: "OpenAI's Dactyl project, which trained a robotic hand to solve a Rubik's Cube, is a famous example of:",
    options: [
      { id: "A", text: "A project that relied exclusively on real-world data." },
      { id: "B", text: "A successful sim-to-real transfer using extensive domain randomization." },
      { id: "C", text: "A project that failed because of the reality gap." },
      { id: "D", text: "A project that used supervised learning instead of reinforcement learning." }
    ],
    correctAnswer: "B",
    explanation: "The Dactyl project is a landmark case study in sim-to-real transfer. The policy was trained entirely in a heavily randomized simulation, allowing it to generalize successfully to the physical robotic hand without any real-world fine-tuning."
  }
]} />
