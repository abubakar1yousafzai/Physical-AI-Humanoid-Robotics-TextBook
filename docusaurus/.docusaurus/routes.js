import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '4e8'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'f84'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '289'),
            routes: [
              {
                path: '/docs/module-01-intro',
                component: ComponentCreator('/docs/module-01-intro', '6de'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01/chapter-01-foundations',
                component: ComponentCreator('/docs/module-01/chapter-01-foundations', '76d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01/chapter-01-quiz',
                component: ComponentCreator('/docs/module-01/chapter-01-quiz', '641'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01/chapter-02-digital-to-physical',
                component: ComponentCreator('/docs/module-01/chapter-02-digital-to-physical', 'ac1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01/chapter-02-quiz',
                component: ComponentCreator('/docs/module-01/chapter-02-quiz', '415'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01/chapter-03-humanoid-landscape',
                component: ComponentCreator('/docs/module-01/chapter-03-humanoid-landscape', 'd4c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01/chapter-03-quiz',
                component: ComponentCreator('/docs/module-01/chapter-03-quiz', '136'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01/chapter-04-quiz',
                component: ComponentCreator('/docs/module-01/chapter-04-quiz', '7dc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01/chapter-04-sensor-systems',
                component: ComponentCreator('/docs/module-01/chapter-04-sensor-systems', '047'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-intro',
                component: ComponentCreator('/docs/module-02-intro', '67f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02/chapter-01-architecture',
                component: ComponentCreator('/docs/module-02/chapter-01-architecture', '72a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02/chapter-01-quiz',
                component: ComponentCreator('/docs/module-02/chapter-01-quiz', 'fba'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02/chapter-02-nodes-topics',
                component: ComponentCreator('/docs/module-02/chapter-02-nodes-topics', '59e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02/chapter-02-quiz',
                component: ComponentCreator('/docs/module-02/chapter-02-quiz', 'c53'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02/chapter-03-packages-python',
                component: ComponentCreator('/docs/module-02/chapter-03-packages-python', 'f8d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02/chapter-03-quiz',
                component: ComponentCreator('/docs/module-02/chapter-03-quiz', 'd5d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02/chapter-04-launch-parameters',
                component: ComponentCreator('/docs/module-02/chapter-04-launch-parameters', '1b8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02/chapter-04-quiz',
                component: ComponentCreator('/docs/module-02/chapter-04-quiz', '720'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-03-intro',
                component: ComponentCreator('/docs/module-03-intro', 'aae'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-03/chapter-01-gazebo-setup',
                component: ComponentCreator('/docs/module-03/chapter-01-gazebo-setup', 'a04'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-03/chapter-01-quiz',
                component: ComponentCreator('/docs/module-03/chapter-01-quiz', '7b9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-03/chapter-02-quiz',
                component: ComponentCreator('/docs/module-03/chapter-02-quiz', '40f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-03/chapter-02-urdf-sdf',
                component: ComponentCreator('/docs/module-03/chapter-02-urdf-sdf', '1a5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-03/chapter-03-physics-sensors',
                component: ComponentCreator('/docs/module-03/chapter-03-physics-sensors', '246'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-03/chapter-03-quiz',
                component: ComponentCreator('/docs/module-03/chapter-03-quiz', 'b94'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-03/chapter-04-quiz',
                component: ComponentCreator('/docs/module-03/chapter-04-quiz', 'e3d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-03/chapter-04-unity-viz',
                component: ComponentCreator('/docs/module-03/chapter-04-unity-viz', '5fa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-04-intro',
                component: ComponentCreator('/docs/module-04-intro', '010'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-04/chapter-01-isaac-sdk-sim',
                component: ComponentCreator('/docs/module-04/chapter-01-isaac-sdk-sim', '5ba'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-04/chapter-01-quiz',
                component: ComponentCreator('/docs/module-04/chapter-01-quiz', 'aaf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-04/chapter-02-perception-manipulation',
                component: ComponentCreator('/docs/module-04/chapter-02-perception-manipulation', 'e13'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-04/chapter-02-quiz',
                component: ComponentCreator('/docs/module-04/chapter-02-quiz', 'c5f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-04/chapter-03-quiz',
                component: ComponentCreator('/docs/module-04/chapter-03-quiz', '56b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-04/chapter-03-reinforcement-learning',
                component: ComponentCreator('/docs/module-04/chapter-03-reinforcement-learning', 'a92'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-04/chapter-04-quiz',
                component: ComponentCreator('/docs/module-04/chapter-04-quiz', 'd7e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-04/chapter-04-sim-to-real',
                component: ComponentCreator('/docs/module-04/chapter-04-sim-to-real', '9cc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-05-intro',
                component: ComponentCreator('/docs/module-05-intro', 'd2c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-05/chapter-01-kinematics-dynamics',
                component: ComponentCreator('/docs/module-05/chapter-01-kinematics-dynamics', '2ac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-05/chapter-01-quiz',
                component: ComponentCreator('/docs/module-05/chapter-01-quiz', '692'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-05/chapter-02-bipedal-locomotion',
                component: ComponentCreator('/docs/module-05/chapter-02-bipedal-locomotion', 'e45'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-05/chapter-02-quiz',
                component: ComponentCreator('/docs/module-05/chapter-02-quiz', '3ec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-05/chapter-03-manipulation-grasping',
                component: ComponentCreator('/docs/module-05/chapter-03-manipulation-grasping', '52f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-05/chapter-03-quiz',
                component: ComponentCreator('/docs/module-05/chapter-03-quiz', 'f4b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-05/chapter-04-human-robot-interaction',
                component: ComponentCreator('/docs/module-05/chapter-04-human-robot-interaction', 'efd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-05/chapter-04-quiz',
                component: ComponentCreator('/docs/module-05/chapter-04-quiz', '661'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-06-intro',
                component: ComponentCreator('/docs/module-06-intro', '08d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-06/chapter-01-integrating-gpt',
                component: ComponentCreator('/docs/module-06/chapter-01-integrating-gpt', '6a7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-06/chapter-01-quiz',
                component: ComponentCreator('/docs/module-06/chapter-01-quiz', '1e5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-06/chapter-02-quiz',
                component: ComponentCreator('/docs/module-06/chapter-02-quiz', '2d7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-06/chapter-02-speech-nlu',
                component: ComponentCreator('/docs/module-06/chapter-02-speech-nlu', '47c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-06/chapter-03-multimodal',
                component: ComponentCreator('/docs/module-06/chapter-03-multimodal', 'dfe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-06/chapter-03-quiz',
                component: ComponentCreator('/docs/module-06/chapter-03-quiz', 'd75'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/preface',
                component: ComponentCreator('/docs/preface', 'b07'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
