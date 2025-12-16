import React from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import CodeBlock from '@theme/CodeBlock';

interface CodeExample {
  label: string;
  language: string;
  code: string;
}

interface CodeBlockProps {
  examples: CodeExample[];
  title?: string;
}

/**
 * Multi-language code block component with tabs
 *
 * Usage:
 * <CodeBlock
 *   title="Publisher Example"
 *   examples={[
 *     { label: 'Python', language: 'python', code: 'import rclpy...' },
 *     { label: 'C++', language: 'cpp', code: '#include <rclcpp/rclcpp.hpp>...' }
 *   ]}
 * />
 */
export default function MultiLangCodeBlock({ examples, title }: CodeBlockProps): JSX.Element {
  return (
    <div className="multi-lang-code-block">
      {title && <h4>{title}</h4>}
      <Tabs groupId="programming-language">
        {examples.map((example, index) => (
          <TabItem key={index} value={example.label.toLowerCase()} label={example.label}>
            <CodeBlock language={example.language}>
              {example.code}
            </CodeBlock>
          </TabItem>
        ))}
      </Tabs>
    </div>
  );
}
