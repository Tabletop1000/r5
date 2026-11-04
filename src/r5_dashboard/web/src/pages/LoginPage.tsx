import { useState, type FormEvent } from 'react';
import { login } from '../api';

interface Props {
  passwordSet: boolean;
  onLoggedIn: () => void;
}

export function LoginPage({ passwordSet, onLoggedIn }: Props) {
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  async function submit(event: FormEvent) {
    event.preventDefault();
    setBusy(true);
    setError(null);
    const result = await login(password);
    setBusy(false);
    if (result.ok) {
      onLoggedIn();
    } else {
      setError(result.message);
      setPassword('');
    }
  }

  return (
    <main className="login">
      <form className="login-card" onSubmit={submit}>
        <div className="brand large" aria-hidden="true">
          R5
        </div>
        <h1>Log in to the robot</h1>
        {!passwordSet ? (
          <p className="error" role="alert">
            No password has been set on this robot yet. On the robot, run{' '}
            <code>ros2 run r5_dashboard_server set_password</code>, then reload this page.
          </p>
        ) : (
          <>
            <label htmlFor="password">Password</label>
            <input
              id="password"
              type="password"
              autoComplete="current-password"
              autoFocus
              required
              value={password}
              onChange={(e) => setPassword(e.target.value)}
            />
            {error && (
              <p className="error" role="alert">
                {error}
              </p>
            )}
            <button type="submit" className="primary" disabled={busy || password.length === 0}>
              {busy ? 'Logging in…' : 'Log in'}
            </button>
          </>
        )}
      </form>
    </main>
  );
}
