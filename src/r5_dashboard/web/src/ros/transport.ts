// A rosbridge WebSocket transport that can be dropped at once (spec NF-3, AS-7).
//
// When WiFi fades out, the WebSocket usually isn't closed: no more data arrives, and the browser
// can take minutes to notice. roslib won't reconnect until the old socket has finished closing.
// abandon() detaches the socket and reports it closed immediately, so a new connection can start.

import { AbstractTransport, type RosbridgeMessage } from 'roslib';

export class WatchedTransport extends AbstractTransport {
  private abandoned = false;
  private readonly socket: WebSocket;
  private readonly onActivity: () => void;

  constructor(socket: WebSocket, onActivity: () => void) {
    super();
    this.socket = socket;
    this.onActivity = onActivity;
    socket.onopen = (e) => {
      this.onActivity();
      this.emit('open', e);
    };
    socket.onclose = (e) => {
      if (this.abandoned) return;
      this.abandoned = true;
      this.emit('close', e);
    };
    socket.onerror = (e) => {
      if (!this.abandoned) this.emit('error', e);
    };
    socket.onmessage = (e) => {
      if (this.abandoned) return;
      this.onActivity();
      this.handleRawMessage(e.data);
    };
  }

  send(message: RosbridgeMessage): void {
    if (!this.abandoned && this.socket.readyState === WebSocket.OPEN) this.socket.send(JSON.stringify(message));
  }

  /** roslib calls this on Ros.close(): drop the socket without waiting for it. */
  close(): void {
    this.abandon('closed');
  }

  /** Stop using this socket now and report it closed, whatever state it is in. */
  abandon(reason: string): void {
    if (this.abandoned) return;
    this.abandoned = true;
    this.socket.onopen = this.socket.onmessage = this.socket.onerror = this.socket.onclose = null;
    try {
      this.socket.close();
    } catch {
      // Already closed.
    }
    this.emit('close', { reason });
  }

  isConnecting(): boolean {
    return !this.abandoned && this.socket.readyState === WebSocket.CONNECTING;
  }

  isOpen(): boolean {
    return !this.abandoned && this.socket.readyState === WebSocket.OPEN;
  }

  isClosing(): boolean {
    return !this.abandoned && this.socket.readyState === WebSocket.CLOSING;
  }

  isClosed(): boolean {
    return this.abandoned || this.socket.readyState === WebSocket.CLOSED;
  }
}
