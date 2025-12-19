import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI Fundamentals',
    icon: '🤖',
    description: (
      <>
        Master the intersection of artificial intelligence and physical systems. 
        Learn how AI algorithms control real-world robotic bodies and interact with environments.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    icon: '🚶',
    description: (
      <>
        Comprehensive coverage of bipedal locomotion, manipulation, and human-robot 
        interaction. Build systems that move and act like humans.
      </>
    ),
  },
  {
    title: 'ROS 2 & Modern Tools',
    icon: '⚙️',
    description: (
      <>
        Hands-on with ROS 2, Gazebo, MoveIt, and industry-standard tools. 
        Practical implementations with real code examples and projects.
      </>
    ),
  },
  {
    title: 'Advanced Perception',
    icon: '👁️',
    description: (
      <>
        Computer vision, sensor fusion, and environmental understanding. 
        Enable robots to see, hear, and comprehend their surroundings.
      </>
    ),
  },
  {
    title: 'AI Integration',
    icon: '🧠',
    description: (
      <>
        Integrate machine learning, reinforcement learning, and cognitive systems 
        into robotic platforms for intelligent decision-making.
      </>
    ),
  },
  {
    title: 'Real-World Projects',
    icon: '🏗️',
    description: (
      <>
        From simulation to physical deployment. Build complete systems 
        for healthcare, manufacturing, and service applications.
      </>
    ),
  },
];

function Feature({title, icon, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4 margin-bottom--lg')}>
      <div className={styles.featureCard}>
        <div className={styles.featureIcon}>
          {icon}
        </div>
        <div className={styles.featureContent}>
          <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <div className={styles.sectionHeader}>
              <Heading as="h2" className={styles.sectionTitle}>
                What You'll Master
              </Heading>
              <p className={styles.sectionSubtitle}>
                Comprehensive coverage of Physical AI and Humanoid Robotics from theory to practice
              </p>
            </div>
          </div>
        </div>
        
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>

        {/* Quick Start Section */}
        <div className={styles.quickStartSection}>
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className={styles.quickStartContent}>
                <Heading as="h2">Ready to Begin Your Journey?</Heading>
                <p>
                  Start with fundamental concepts and progressively build advanced humanoid systems. 
                  Each chapter includes hands-on exercises and real-world projects.
                </p>
                <div className={styles.quickStartButtons}>
                  <a className={styles.primaryButton} href="/docs/intro">
                    Start Reading Now
                  </a>
                  
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}