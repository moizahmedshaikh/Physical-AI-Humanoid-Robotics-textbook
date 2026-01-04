// index.js - UPDATED REACT COMPONENT WITH MOUSE INTERACTION

import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function AdvancedRobot() {
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });
  const [robotStyle, setRobotStyle] = useState({});

    const robotRef = useRef(null);

  useEffect(() => {
    const el = robotRef.current;
    if (!el) return;

    let handActive = false;
    function onMove(e) {
      const rect = el.getBoundingClientRect();
      const cx = rect.left + rect.width/2;
      const cy = rect.top + rect.height/2;
      const dx = e.clientX - cx;
      const dy = e.clientY - cy;

      // rotate subtly based on pointer
      const rotY = Math.max(-12, Math.min(12, dx / 18));
      const rotX = Math.max(-8, Math.min(8, -dy / 40));
      el.style.setProperty('--rotY', `${rotY}deg`);
      el.style.setProperty('--rotX', `${rotX}deg`);

      // eye follow - small translation
      const maxEye = 4; // px
      const eyeX = Math.max(-maxEye, Math.min(maxEye, dx / 80));
      const eyeY = Math.max(-maxEye, Math.min(maxEye, dy / 160));
      // set on root so CSS .eye uses them
      el.querySelectorAll('.eye').forEach((eye) => {
        eye.style.setProperty('--eye-x', `${eyeX}px`);
        eye.style.setProperty('--eye-y', `${eyeY}px`);
      });

      // hand active when pointer close to robot
      const dist = Math.hypot(dx, dy);
      const active = dist < rect.width * 0.9 ? '1' : '0';
      if (el.getAttribute('data-hand-active') !== active) el.setAttribute('data-hand-active', active);
    }

    function onLeave() {
      el.style.setProperty('--rotY', `0deg`);
      el.style.setProperty('--rotX', `0deg`);
      el.querySelectorAll('.eye').forEach((eye) => {
        eye.style.setProperty('--eye-x', `0px`);
        eye.style.setProperty('--eye-y', `0px`);
      });
      el.setAttribute('data-hand-active', '0');
    }

    window.addEventListener('mousemove', onMove);
    window.addEventListener('touchmove', onMove, {passive:true});
    window.addEventListener('mouseleave', onLeave);

    return () => {
      window.removeEventListener('mousemove', onMove);
      window.removeEventListener('touchmove', onMove);
      window.removeEventListener('mouseleave', onLeave);
    };
  }, []);

  return (
    <div 
      className={styles.robotContainer}
      ref={robotRef}
    >
      <div className={styles.energyPulse} />
      
      <div className={styles.robot} style={robotStyle.robot}>
        {/* Robot Base */}
        <div className={styles.robotBase} />
        
        {/* Robot Body */}
        <div className={styles.robotBody} style={robotStyle.body}>
          {/* Body details */}
          <div className={styles.bodyPanel} />
          <div className={styles.bodyPanel} />
          <div className={styles.bodyPanel} />
        </div>
        
        {/* Robot Head */}
        <div className={styles.robotHead} style={robotStyle.head}>
          <div className={styles.robotEyes}>
            <div className={styles.robotEye} />
            <div className={styles.robotEye} />
          </div>
        </div>
        
        {/* Left Arm with Joints */}
        <div className={clsx(styles.robotArm, styles.robotLeftArm)} style={robotStyle.leftArm}>
          <div className={clsx(styles.robotJoint, styles.jointShoulder)} />
          <div className={clsx(styles.robotJoint, styles.jointElbow)} />
        </div>
        
        {/* Right Arm with Joints */}
        <div className={clsx(styles.robotArm, styles.robotRightArm)} style={robotStyle.rightArm}>
          <div className={clsx(styles.robotJoint, styles.jointShoulder)} />
          <div className={clsx(styles.robotJoint, styles.jointElbow)} />
        </div>
        
        {/* Left Leg with Joints */}
        <div className={clsx(styles.robotLeg, styles.robotLeftLeg)} style={robotStyle.leftLeg}>
          <div className={clsx(styles.robotJoint, styles.jointHip)} />
          <div className={clsx(styles.robotJoint, styles.jointKnee)} />
        </div>
        
        {/* Right Leg with Joints */}
        <div className={clsx(styles.robotLeg, styles.robotRightLeg)} style={robotStyle.rightLeg}>
          <div className={clsx(styles.robotJoint, styles.jointHip)} />
          <div className={clsx(styles.robotJoint, styles.jointKnee)} />
        </div>
      </div>
    </div>
  );
}

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      {/* Animated Orbs */}
      <div className={styles.heroOrb} />
      <div className={styles.heroOrb} />
      <div className={styles.heroOrb} />
      
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className={styles.heroTitle}>
              Physical AI &  
              <span> Humanoid Robotics</span>
            </Heading>
            <p className={styles.heroSubtitle}>
              The Complete Guide to Building Intelligent Robotic Systems That Think, Learn, and Interact
            </p>
            
            <div className={styles.heroStats}>
              <div className={styles.stat}>
                <span className={styles.statNumber}>12</span>
                <span className={styles.statLabel}>Chapters</span>
              </div>
              <div className={styles.stat}>
                <span className={styles.statNumber}>50+</span>
                <span className={styles.statLabel}>Projects</span>
              </div>
              <div className={styles.stat}>
                <span className={styles.statNumber}>100+</span>
                <span className={styles.statLabel}>Code Examples</span>
              </div>
              <div className={styles.stat}>
                <span className={styles.statNumber}>24/7</span>
                <span className={styles.statLabel}>Support</span>
              </div>
            </div>
            
            <div className={styles.heroButtons}>
              <Link
                className={styles.heroButtonPrimary}
                to="/docs/intro">
                <span className={styles.buttonGlow}>Start Reading Free</span>
              </Link>
            </div>
          </div>
          
          {/* Advanced Robot Animation */}
          <div className={styles.heroVisual}>
            <AdvancedRobot />
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  
  return (
    <Layout
      title="Physical AI & Humanoid Robotics - Complete Guide"
      description="Master Physical AI and Humanoid Robotics with comprehensive tutorials, projects, and real-world applications. Learn ROS 2, AI integration, and advanced robotics systems.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}