import type { ReactNode } from 'react';

export function Card({ title, actions, children, className }: { title: string; actions?: ReactNode; children: ReactNode; className?: string }) {
  return (
    <section className={`card${className ? ` ${className}` : ''}`}>
      <div className="card-head">
        <h2>{title}</h2>
        {actions}
      </div>
      {children}
    </section>
  );
}

export type Tone = 'good' | 'warn' | 'bad' | 'neutral';

/** One labelled value. Missing values read "Not available". */
export function Reading({ label, value, tone, hint }: { label: string; value: ReactNode | null; tone?: Tone; hint?: string }) {
  const missing = value === null || value === undefined;
  return (
    <div className="reading">
      <dt>{label}</dt>
      <dd className={missing ? 'missing' : tone ? `tone-text-${tone}` : undefined} title={hint}>
        {missing ? 'Not available' : value}
      </dd>
    </div>
  );
}

export function Readings({ children }: { children: ReactNode }) {
  return <dl className="readings">{children}</dl>;
}
