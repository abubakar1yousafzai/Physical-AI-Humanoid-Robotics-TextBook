---
id: chapter-02-quiz
title: "Chapter 2: Quiz"
sidebar_label: "Quiz"
---
import Quiz from '@site/src/components/Quiz';

# Chapter 2: Bipedal Locomotion Quiz

<Quiz questions={[
  {
    id: 1,
    question: "What is the most critical aspect of bipedal locomotion?",
    options: [
      { id: "A", text: "Moving the legs as fast as possible." },
      { id: "B", text: "Maintaining balance." },
      { id: "C", text: "Keeping the robot's arms perfectly still." },
      { id: "D", text: "Having the heaviest possible feet." }
    ],
    correctAnswer: "B",
    explanation: "Maintaining balance is the most critical aspect of bipedal locomotion, as a walking robot is inherently unstable and constantly on the verge of falling. It must continuously adjust its posture and foot placement to keep its center of mass within its base of support."
  },
  {
    id: 2,
    question: "What is the Zero Moment Point (ZMP)?",
    options: [
      { id: "A", text: "The point on the robot's body with the least mass." },
      { id: "B", text: "The point on the ground where the net moment of the inertial and gravity forces is zero." },
      { id: "C", text: "The robot's center of mass." },
      { id: "D", text: "The point in the air where the robot is aiming to step." }
    ],
    correctAnswer: "B",
    explanation: "The Zero Moment Point (ZMP) is a crucial concept in bipedal locomotion, representing the point where all resultant forces act on the ground. For a robot to maintain stable contact and not tip over, its ZMP must always remain within its support polygon."
  },
  {
    id: 3,
    question: "What is a major disadvantage of using a pre-computed trajectory for walking?",
    options: [
      { id: "A", text: "It is computationally very expensive to execute." },
      { id: "B", text: "It is not robust to disturbances or unexpected changes in the environment." },
      { id: "C", text: "It only works for robots with one leg." },
      { id: "D", text: "It requires a very powerful computer to generate." }
    ],
    correctAnswer: "B",
    explanation: "Pre-computed trajectories are typically open-loop or use minimal feedback, meaning they struggle to adapt to unforeseen circumstances like uneven terrain or external pushes. This lack of robustness significantly limits their applicability in dynamic, real-world environments."
  },
  {
    id: 4,
    question: "What is Model Predictive Control (MPC)?",
    options: [
      { id: "A", text: "A method for predicting the robot's future battery life." },
      { id: "B", text: "A control strategy that uses a model of the robot to predict its future behavior and optimize its actions." },
      { id: "C", text: "A type of sensor that predicts when a collision is about to occur." },
      { id: "D", text: "A machine learning model that predicts the user's intent." }
    ],
    correctAnswer: "B",
    explanation: "Model Predictive Control (MPC) is an advanced control strategy that optimizes a sequence of control actions over a short future horizon, taking into account the robot's dynamics and constraints. This iterative optimization makes it highly robust to disturbances and capable of handling complex, non-linear systems."
  },
  {
    id: 5,
    question: "In the provided Python code, what does the `generate_gait` function do?",
    options: [
      { id: "A", text: "It calculates the robot's ZMP." },
      { id: "B", text: "It generates a simple walking gait using sine waves." },
      { id: "C", text: "It plans a collision-free path for the robot." },
      { id: "D", text: "It simulates the robot's dynamics." }
    ],
    correctAnswer: "B",
    explanation: "The `generate_gait` function creates a basic, periodic motion pattern for a robot's legs using sine waves. This serves as a fundamental example of trajectory generation for bipedal locomotion, providing a repeatable movement pattern for walking."
  }
]} />
