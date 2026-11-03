// The single link to the robot's ROS system, through the dashboard server's /ws (spec D2, D7).
// Reconnects by itself and keeps subscriptions across reconnects (spec NF-3).

import { Ros, Topic } from 'roslib';
import { reconnectDelayMs } from '../status';

export type LinkStatus = 'connecting' | 'connected' | 'disconnected';

export interface LinkState {
  status: LinkStatus;
  /** When the link was last known to be up (ms since epoch), or null if it never was. */
  lastContact: number | null;
}

export interface RobotConnectionOptions {
  url: string;
  /** Asked after each drop; resolve false when the session has ended, to stop retrying. */
  isLoggedIn: () => Promise<boolean>;
  onLoggedOut: () => void;
}

export class RobotConnection {
  private readonly ros = new Ros();
  private state: LinkState = { status: 'connecting', lastContact: null };
  private readonly listeners = new Set<() => void>();
  private attempt = 0;
  private timer: ReturnType<typeof setTimeout> | undefined;
  private stopped = false;
  private readonly options: RobotConnectionOptions;

  constructor(options: RobotConnectionOptions) {
    this.options = options;
    this.ros.on('connection', () => {
      this.attempt = 0;
      this.setState({ status: 'connected', lastContact: Date.now() });
    });
    this.ros.on('close', () => this.handleDrop());
    // 'error' is always followed by 'close'; listening stops it being thrown as unhandled.
    this.ros.on('error', () => undefined);
  }

  /** Current state. Stable object between changes, for useSyncExternalStore. */
  getState = (): LinkState => this.state;

  subscribe = (listener: () => void): (() => void) => {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  };

  start(): void {
    this.stopped = false;
    void this.connect();
  }

  stop(): void {
    this.stopped = true;
    clearTimeout(this.timer);
    this.ros.close();
  }

  topic<T>(name: string, messageType: string, throttleMs = 0): Topic<T> {
    return this.ros.Topic<T>({ name, messageType, throttle_rate: throttleMs });
  }

  private async connect(): Promise<void> {
    if (this.stopped) return;
    this.setState({ ...this.state, status: this.state.lastContact ? 'disconnected' : 'connecting' });
    try {
      await this.ros.connect(this.options.url);
    } catch {
      // Usually a 'close' event also arrives; handleDrop replaces any timer, so only one retry
      // is ever scheduled.
      this.handleDrop();
    }
  }

  private handleDrop(): void {
    if (this.stopped) return;
    const lastContact = this.state.status === 'connected' ? Date.now() : this.state.lastContact;
    this.setState({ status: lastContact ? 'disconnected' : 'connecting', lastContact });
    clearTimeout(this.timer);
    const delay = reconnectDelayMs(this.attempt++);
    this.timer = setTimeout(() => void this.retry(), delay);
  }

  private async retry(): Promise<void> {
    if (this.stopped) return;
    let loggedIn = true;
    try {
      loggedIn = await this.options.isLoggedIn();
    } catch {
      // The robot is unreachable, so we can't tell. Keep trying.
    }
    if (!loggedIn) {
      this.stop();
      this.options.onLoggedOut();
      return;
    }
    await this.connect();
  }

  private setState(next: LinkState): void {
    if (next.status === this.state.status && next.lastContact === this.state.lastContact) return;
    this.state = next;
    for (const listener of this.listeners) listener();
  }
}

/** The WebSocket address on the same host and port the app was loaded from. */
export function defaultRosUrl(location: Location = window.location): string {
  const scheme = location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${scheme}//${location.host}/ws`;
}
