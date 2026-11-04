import { useState, type FormEvent } from 'react';
import { changePassword } from '../../api';
import { Card } from '../../components/Card';

const MIN_LENGTH = 8;

/** Change the dashboard password (spec NF-10). Other devices are logged out. */
export function PasswordCard() {
  const [current, setCurrent] = useState('');
  const [next, setNext] = useState('');
  const [repeat, setRepeat] = useState('');
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState<{ ok: boolean; text: string } | null>(null);

  const mismatch = repeat.length > 0 && next !== repeat;
  const canSubmit = current.length > 0 && next.length >= MIN_LENGTH && next === repeat && !busy;

  async function submit(e: FormEvent) {
    e.preventDefault();
    setBusy(true);
    setMessage(null);
    const result = await changePassword(current, next);
    setBusy(false);
    if (result.ok) {
      setCurrent('');
      setNext('');
      setRepeat('');
      setMessage({ ok: true, text: 'Password changed. Other devices have been logged out.' });
    } else {
      setMessage({ ok: false, text: result.message });
    }
  }

  return (
    <Card title="Dashboard password">
      <form className="form" onSubmit={submit}>
        <label>
          Current password
          <input type="password" autoComplete="current-password" value={current} onChange={(e) => setCurrent(e.target.value)} />
        </label>
        <label>
          New password <span className="muted small">(at least {MIN_LENGTH} characters)</span>
          <input type="password" autoComplete="new-password" value={next} onChange={(e) => setNext(e.target.value)} />
        </label>
        <label>
          Repeat new password
          <input type="password" autoComplete="new-password" value={repeat} onChange={(e) => setRepeat(e.target.value)} aria-invalid={mismatch} />
        </label>
        {mismatch && <p className="error">The new passwords don't match.</p>}
        {message && (
          <p className={message.ok ? 'note' : 'error'} role="status">
            {message.text}
          </p>
        )}
        <div>
          <button type="submit" className="primary" disabled={!canSubmit}>
            {busy ? 'Changing…' : 'Change password'}
          </button>
        </div>
      </form>
    </Card>
  );
}
