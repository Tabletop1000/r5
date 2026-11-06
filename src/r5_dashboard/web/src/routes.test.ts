import { describe, expect, it } from 'vitest';
import { DEFAULT_PAGE, hashForPage, pageFromHash } from './routes';

describe('pageFromHash', () => {
  it('reads known pages', () => {
    expect(pageFromHash('#/robot')).toBe('robot');
    expect(pageFromHash('#robot')).toBe('robot');
    expect(pageFromHash('#/spray/area')).toBe('spray');
    expect(pageFromHash('#/settings')).toBe('settings');
  });
  it('falls back to the default page', () => {
    expect(pageFromHash('')).toBe(DEFAULT_PAGE);
    expect(pageFromHash('#/nonsense')).toBe(DEFAULT_PAGE);
  });
  it('round-trips with hashForPage', () => {
    expect(pageFromHash(hashForPage('mow'))).toBe('mow');
  });
});
