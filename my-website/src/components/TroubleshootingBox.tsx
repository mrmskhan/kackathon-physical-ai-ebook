import React, { ReactNode } from 'react';
import CodeBlock from '@theme/CodeBlock';
import styles from './TroubleshootingBox.module.css';

interface TroubleshootingBoxProps {
  issue: string;
  symptom?: string | ReactNode;
  cause?: string | ReactNode;
  solution: string | ReactNode;
  verification?: string;
  children?: ReactNode;
}

/**
 * Structured troubleshooting component
 *
 * Usage:
 * <TroubleshootingBox
 *   issue="Package not found"
 *   symptom="ModuleNotFoundError: No module named 'rclpy'"
 *   cause="ROS 2 environment not sourced"
 *   solution="source /opt/ros/humble/setup.bash"
 *   verification="python3 -c 'import rclpy'"
 * />
 */
export default function TroubleshootingBox({
  issue,
  symptom,
  cause,
  solution,
  verification,
  children
}: TroubleshootingBoxProps): JSX.Element {
  return (
    <div className={styles.troubleshootingBox}>
      <div className={styles.troubleshootingHeader}>
        <span className={styles.troubleshootingIcon}>🔧</span>
        <strong className={styles.troubleshootingTitle}>{issue}</strong>
      </div>

      {symptom && (
        <div className={styles.troubleshootingSection}>
          <strong className={styles.sectionLabel}>Symptom:</strong>
          {typeof symptom === 'string' ? (
            <CodeBlock language="text">{symptom}</CodeBlock>
          ) : (
            <div className={styles.sectionContent}>{symptom}</div>
          )}
        </div>
      )}

      {cause && (
        <div className={styles.troubleshootingSection}>
          <strong className={styles.sectionLabel}>Root Cause:</strong>
          <div className={styles.sectionContent}>{cause}</div>
        </div>
      )}

      <div className={styles.troubleshootingSection}>
        <strong className={styles.sectionLabel}>Solution:</strong>
        {typeof solution === 'string' ? (
          <CodeBlock language="bash">{solution}</CodeBlock>
        ) : (
          <div className={styles.sectionContent}>{solution}</div>
        )}
      </div>

      {verification && (
        <div className={styles.troubleshootingSection}>
          <strong className={styles.sectionLabel}>Verification:</strong>
          <CodeBlock language="bash">{verification}</CodeBlock>
        </div>
      )}

      {children && (
        <div className={styles.troubleshootingSection}>
          {children}
        </div>
      )}
    </div>
  );
}
