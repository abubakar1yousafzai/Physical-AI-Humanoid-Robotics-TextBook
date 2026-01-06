// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    'preface',
    {
      type: 'category',
      label: 'MODULE 1: Introduction to Physical AI',
      link: { type: 'doc', id: 'module-01-intro' },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Foundations of Physical AI',
          link: { type: 'doc', id: 'module-01/chapter-01-foundations' },
          items: ['module-01/chapter-01-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 2: From Digital AI to Robots',
          link: { type: 'doc', id: 'module-01/chapter-02-digital-to-physical' },
          items: ['module-01/chapter-02-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 3: Humanoid Robotics Landscape',
          link: { type: 'doc', id: 'module-01/chapter-03-humanoid-landscape' },
          items: ['module-01/chapter-03-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 4: Sensor Systems',
          link: { type: 'doc', id: 'module-01/chapter-04-sensor-systems' },
          items: ['module-01/chapter-04-quiz'],
        },
      ],
    },
    {
      type: 'category',
      label: 'MODULE 2: ROS 2 Fundamentals',
      link: { type: 'doc', id: 'module-02-intro' },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Architecture',
          link: { type: 'doc', id: 'module-02/chapter-01-architecture' },
          items: ['module-02/chapter-01-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 2: Nodes, Topics, Services, & Actions',
          link: { type: 'doc', id: 'module-02/chapter-02-nodes-topics' },
          items: ['module-02/chapter-02-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 3: Building Packages with Python',
          link: { type: 'doc', id: 'module-02/chapter-03-packages-python' },
          items: ['module-02/chapter-03-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 4: Launch Files & Parameters',
          link: { type: 'doc', id: 'module-02/chapter-04-launch-parameters' },
          items: ['module-02/chapter-04-quiz'],
        },
      ],
    },
    {
      type: 'category',
      label: 'MODULE 3: Robot Simulation with Gazebo',
      link: { type: 'doc', id: 'module-03-intro' },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Setting Up the Gazebo Simulation Environment',
          link: { type: 'doc', id: 'module-03/chapter-01-gazebo-setup' },
          items: ['module-03/chapter-01-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 2: Creating Robot Models with URDF and SDF',
          link: { type: 'doc', id: 'module-03/chapter-02-urdf-sdf' },
          items: ['module-03/chapter-02-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 3: Simulating Physics and Sensors',
          link: { type: 'doc', id: 'module-03/chapter-03-physics-sensors' },
          items: ['module-03/chapter-03-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 4: High-Fidelity Visualization with Unity',
          link: { type: 'doc', id: 'module-03/chapter-04-unity-viz' },
          items: ['module-03/chapter-04-quiz'],
        },
      ],
    },
    {
      type: 'category',
      label: 'MODULE 4: NVIDIA Isaac Platform',
      link: { type: 'doc', id: 'module-04-intro' },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Introduction to Isaac SDK and Isaac Sim',
          link: { type: 'doc', id: 'module-04/chapter-01-isaac-sdk-sim' },
          items: ['module-04/chapter-01-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 2: AI-Powered Perception and Manipulation',
          link: { type: 'doc', id: 'module-04/chapter-02-perception-manipulation' },
          items: ['module-04/chapter-02-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 3: Reinforcement Learning for Robotics',
          link: { type: 'doc', id: 'module-04/chapter-03-reinforcement-learning' },
          items: ['module-04/chapter-03-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 4: Sim-to-Real Transfer',
          link: { type: 'doc', id: 'module-04/chapter-04-sim-to-real' },
          items: ['module-04/chapter-04-quiz'],
        },
      ],
    },
    {
      type: 'category',
      label: 'MODULE 5: Humanoid Robot Development',
      link: { type: 'doc', id: 'module-05-intro' },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Humanoid Robot Kinematics and Dynamics',
          link: { type: 'doc', id: 'module-05/chapter-01-kinematics-dynamics' },
          items: ['module-05/chapter-01-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 2: Bipedal Locomotion',
          link: { type: 'doc', id: 'module-05/chapter-02-bipedal-locomotion' },
          items: ['module-05/chapter-02-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 3: Manipulation and Grasping',
          link: { type: 'doc', id: 'module-05/chapter-03-manipulation-grasping' },
          items: ['module-05/chapter-03-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 4: Human-Robot Interaction',
          link: { type: 'doc', id: 'module-05/chapter-04-human-robot-interaction' },
          items: ['module-05/chapter-04-quiz'],
        },
      ],
    },
    {
      type: 'category',
      label: 'MODULE 6: Conversational Robotics',
      link: { type: 'doc', id: 'module-06-intro' },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Integrating GPT Models into Robots',
          link: { type: 'doc', id: 'module-06/chapter-01-integrating-gpt' },
          items: ['module-06/chapter-01-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 2: Speech Recognition and Natural Language Understanding',
          link: { type: 'doc', id: 'module-06/chapter-02-speech-nlu' },
          items: ['module-06/chapter-02-quiz'],
        },
        {
          type: 'category',
          label: 'Chapter 3: Multi-modal Interaction',
          link: { type: 'doc', id: 'module-06/chapter-03-multimodal' },
          items: ['module-06/chapter-03-quiz'],
        },
      ],
    },
  ],
};

export default sidebars;
