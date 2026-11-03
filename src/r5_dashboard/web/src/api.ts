// Calls to the dashboard server's login API (spec NF-10). The session lives in an HttpOnly cookie,
// so the app never sees or stores the token itself.

export interface SessionInfo {
  authenticated: boolean;
  passwordSet: boolean;
}

export async function getSession(): Promise<SessionInfo> {
  const resp = await fetch('/api/session', { credentials: 'same-origin', cache: 'no-store' });
  if (!resp.ok) throw new Error(`The robot answered ${resp.status}.`);
  return (await resp.json()) as SessionInfo;
}

export type LoginResult = { ok: true } | { ok: false; message: string };

export async function login(password: string): Promise<LoginResult> {
  let resp: Response;
  try {
    resp = await fetch('/api/login', {
      method: 'POST',
      credentials: 'same-origin',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ password }),
    });
  } catch {
    return { ok: false, message: "Can't reach the robot. Check the WiFi link and try again." };
  }
  if (resp.ok) return { ok: true };
  let message = `Login failed (${resp.status}).`;
  try {
    const body = (await resp.json()) as { error?: string };
    if (body.error) message = body.error;
  } catch {
    // Keep the generic message.
  }
  return { ok: false, message };
}

export async function logout(): Promise<void> {
  try {
    await fetch('/api/logout', { method: 'POST', credentials: 'same-origin' });
  } catch {
    // The cookie is dropped by the server; if the robot is unreachable there is nothing more to do.
  }
}

export async function changePassword(current: string, next: string): Promise<LoginResult> {
  let resp: Response;
  try {
    resp = await fetch('/api/password', {
      method: 'POST',
      credentials: 'same-origin',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ current, new: next }),
    });
  } catch {
    return { ok: false, message: "Can't reach the robot. Check the WiFi link and try again." };
  }
  if (resp.ok) return { ok: true };
  try {
    const body = (await resp.json()) as { error?: string };
    return { ok: false, message: body.error ?? `Failed (${resp.status}).` };
  } catch {
    return { ok: false, message: `Failed (${resp.status}).` };
  }
}

export interface MapLayerConfig {
  id: string;
  name: string;
  tiles: string[];
  tileSize: number;
  maxzoom: number;
  attribution: string;
}

export interface MapConfig {
  layers: MapLayerConfig[];
}

let mapConfig: Promise<MapConfig> | null = null;

/** Tile sources set on the robot (spec NF-11, NF-12). Fetched once per page load. */
export function getMapConfig(): Promise<MapConfig> {
  mapConfig ??= fetch('/api/map-config', { credentials: 'same-origin' })
    .then((r) => (r.ok ? (r.json() as Promise<MapConfig>) : { layers: [] }))
    .catch(() => ({ layers: [] }));
  return mapConfig;
}
