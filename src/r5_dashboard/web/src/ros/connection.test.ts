import { describe, expect, it } from 'vitest';
import { defaultRosUrl, watchdogAction } from './connection';

describe('defaultRosUrl', () => {
  it('uses wss on https, on the same host and port', () => {
    expect(defaultRosUrl({ protocol: 'https:', host: 'r4:8443' } as Location)).toBe('wss://r4:8443/ws');
  });
  it('uses ws on plain http (Vite dev server)', () => {
    expect(defaultRosUrl({ protocol: 'http:', host: 'localhost:5173' } as Location)).toBe('ws://localhost:5173/ws');
  });
});

describe('watchdogAction', () => {
  const base = { connected: true, connecting: false, sinceActivityMs: 0, sinceAttemptMs: 0, pinging: false };
  it('does nothing while data flows', () => {
    expect(watchdogAction(base)).toBe('none');
  });
  it('pings after a quiet spell, once', () => {
    expect(watchdogAction({ ...base, sinceActivityMs: 3000 })).toBe('ping');
    expect(watchdogAction({ ...base, sinceActivityMs: 3000, pinging: true })).toBe('none');
  });
  it('drops a link that has gone silent, even if the socket looks open', () => {
    expect(watchdogAction({ ...base, sinceActivityMs: 9000, pinging: true })).toBe('drop');
  });
  it('gives up on a connection attempt that hangs', () => {
    expect(watchdogAction({ ...base, connected: false, connecting: true, sinceAttemptMs: 5000 })).toBe('none');
    expect(watchdogAction({ ...base, connected: false, connecting: true, sinceAttemptMs: 11000 })).toBe('drop');
  });
  it('leaves a closed link to the reconnect timer', () => {
    expect(watchdogAction({ ...base, connected: false, sinceActivityMs: 60000 })).toBe('none');
  });
});
