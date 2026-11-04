import type { RosTime } from '../ros/topics';

export function formatDate(t: RosTime): string {
  if (!t.sec) return '—';
  return new Date(t.sec * 1000).toLocaleDateString([], { day: 'numeric', month: 'short', year: 'numeric' });
}
