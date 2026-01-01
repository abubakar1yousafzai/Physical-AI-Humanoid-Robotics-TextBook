import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  useEffect(() => {
    // Force dark theme for the landing page
    document.documentElement.setAttribute('data-theme', 'dark');

    // Typing effect logic
    const subtitle = document.querySelector('.hero-subtitle');
    if (subtitle) {
      const text = "Master the Future of Robotics and Embodied Intelligence";
      subtitle.textContent = '';
      let i = 0;
      const type = () => {
        if (i < text.length) {
          subtitle.textContent += text.charAt(i);
          i++;
          setTimeout(type, 50);
        }
      };
      type();
    }

    // Scroll Reveal Logic
    const observerOptions = {
      threshold: 0.1
    };

    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.classList.add('visible');
          
          // Counter animation logic
          if (entry.target.classList.contains('stat-card')) {
            const valueDisplay = entry.target.querySelector('.stat-value');
            const target = parseInt(valueDisplay.getAttribute('data-target'));
            let current = 0;
            const increment = target / 50; // Adjust for speed
            const updateCounter = () => {
              current += increment;
              if (current < target) {
                valueDisplay.textContent = Math.ceil(current);
                requestAnimationFrame(updateCounter);
              } else {
                valueDisplay.textContent = target + (target >= 1000 ? '+' : '');
              }
            };
            updateCounter();
          }
        }
      });
    }, observerOptions);

    document.querySelectorAll('.reveal').forEach(el => observer.observe(el));

    return () => observer.disconnect();
  }, []);


  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Master the Future of Robotics and Embodied Intelligence"
    >
      <main>
        <section className="hero-section">
          <div className="hero-content">
            <h1 className="neon-title fade-in stagger-1">
              Physical AI & <br /> Humanoid Robotics
            </h1>
            <p className="hero-subtitle">
              Master the Future of Robotics and Embodied Intelligence
            </p>
            <div className="hero-buttons fade-in stagger-3">
              <a href="/docs/preface" className="neon-btn primary">
                Start Learning
              </a>
              <a href="#modules" className="neon-btn secondary">
                View Modules
              </a>
            </div>
          </div>
          <div className="scroll-indicator">
            <div className="mouse">
              <div className="wheel"></div>
            </div>
            <div>
              <span className="m_scroll_arrows unu"></span>
              <span className="m_scroll_arrows doi"></span>
              <span className="m_scroll_arrows trei"></span>
            </div>
          </div>
        </section>

        <section id="modules" className="section modules-section">
          <div className="container">
            <h2 className="section-title neon-text">Learning Modules</h2>
            <div className="modules-grid">
              {[
                { id: '01', title: 'Introduction to Physical AI', chapters: 4, desc: 'Lay the foundation for understanding embodied intelligence and the future of robotics.' },
                { id: '02', title: 'ROS 2 Fundamentals', chapters: 4, desc: 'Master the Robot Operating System 2, the industry standard for building robotic applications.' },
                { id: '03', title: 'Robot Simulation with Gazebo', chapters: 4, desc: 'Learn to build and test robots in high-fidelity virtual environments before physical deployment.' },
                { id: '04', title: 'Introduction to the NVIDIA Isaac Platform', chapters: 4, desc: 'Harness the power of AI-driven simulation and acceleration with NVIDIA Isaac Sim.' },
                { id: '05', title: 'Introduction to Humanoid Robot Development', chapters: 4, desc: 'Explore the complexities of bipedal locomotion and humanoid form factors.' },
                { id: '06', title: 'Conversational Robotics', chapters: 3, desc: 'Integrate LLMs and natural language processing to create robots that can interact and reason.' }
              ].map((module, idx) => (
                <div key={idx} className="module-card glass fade-in">
                  <div className="module-number">{module.id}</div>
                  <h3 className="module-title">{module.title}</h3>
                  <p className="module-meta">{module.chapters} Chapters</p>
                  <div className="module-preview">
                    {module.desc}
                  </div>
                  <a href={`/docs/module-${module.id}-intro`} className="module-btn">
                    Start Module
                  </a>
                </div>
              ))}
            </div>
          </div>
        </section>

        <section className="section features-section">
          <div className="container">
            <h2 className="section-title neon-text">Key Features</h2>
            <div className="features-grid">
              {[
                { icon: '📚', title: 'Interactive Textbook', desc: 'Engage with dynamic content and code snippets.' },
                { icon: '🤖', title: 'AI Chat Assistant', desc: 'Get instant help from our robotics-trained AI.' },
                { icon: '🎓', title: '6 Complete Modules', desc: 'Structured learning from basics to advanced AI.' },
                { icon: '💾', title: 'Progress Tracking', desc: 'Save your learning journey (Coming soon).' },
                { icon: '🔍', title: 'Smart Search', desc: 'Find any robotics concept in seconds.' },
                { icon: '📱', title: 'Mobile Responsive', desc: 'Learn on the go with a seamless mobile experience.' }
              ].map((feature, idx) => (
                <div key={idx} className="feature-card reveal">
                  <div className="feature-icon">{feature.icon}</div>
                  <h3 className="feature-title">{feature.title}</h3>
                  <p className="feature-desc">{feature.desc}</p>
                </div>
              ))}
            </div>
          </div>
        </section>

        <section className="section stats-section">
          <div className="container">
            <div className="stats-grid">
              {[
                { label: 'Modules', value: 6 },
                { label: 'Chapters', value: 23 },
                { label: 'Concepts', value: 1000 },
                { label: 'AI Responses', value: 5000 }
              ].map((stat, idx) => (
                <div key={idx} className="stat-card reveal">
                  <div className="stat-value neon-text" data-target={stat.value}>0</div>
                  <div className="stat-label">{stat.label}</div>
                </div>
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>



  );
}