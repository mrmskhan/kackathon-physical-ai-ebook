import React, { ReactNode } from 'react';
import styles from './CalloutBox.module.css';

type CalloutType = 'info' | 'warning' | 'tip' | 'prerequisite' | 'note';

interface CalloutBoxProps {
  type?: CalloutType;
  title?: string;
  children: ReactNode;
}

const icons = {
  info: 'ℹ️',
  warning: '⚠️',
  tip: '💡',
  prerequisite: '📋',
  note: '📝',
};

const defaultTitles = {
  info: 'Info',
  warning: 'Warning',
  tip: 'Tip',
  prerequisite: 'Prerequisites',
  note: 'Note',
};

/**
 * Callout box for highlighting important information
 *
 * Usage:
 * <CalloutBox type="prerequisite" title="Before You Begin">
 *   - Completed Module 1
 *   - ROS 2 Humble installed
 * </CalloutBox>
 */
export default function CalloutBox({
  type = 'info',
  title,
  children
}: CalloutBoxProps): JSX.Element {
  const displayTitle = title || defaultTitles[type];
  const icon = icons[type];

  return (
    <div className={`${styles.calloutBox} ${styles[type]}`}>
      <div className={styles.calloutHeader}>
        <span className={styles.calloutIcon}>{icon}</span>
        <strong className={styles.calloutTitle}>{displayTitle}</strong>
      </div>
      <div className={styles.calloutContent}>
        {children}
      </div>
    </div>
  );
}
