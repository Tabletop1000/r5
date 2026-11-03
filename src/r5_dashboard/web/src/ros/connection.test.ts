import { describe, expect, it } from 'vitest';
import { defaultRosUrl } from './connection';

describe('defaultRosUrl', () => {
  it('uses wss on https, on the same host and port', () => {
    expect(defaultRosUrl({ protocol: 'https:', host: 'r4:8443' } as Location)).toBe('wss://r4:8443/ws');
  });
  it('uses ws on plain http (Vite dev server)', () => {
    expect(defaultRosUrl({ protocol: 'http:', host: 'localhost:5173' } as Location)).toBe('ws://localhost:5173/ws');
  });
});
