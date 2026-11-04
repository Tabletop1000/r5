import { useCallback, useEffect, useState } from 'react';
import { getSession, logout } from './api';
import { Dashboard } from './Dashboard';
import { LoginPage } from './pages/LoginPage';

type View =
  | { kind: 'checking' }
  | { kind: 'unreachable'; message: string }
  | { kind: 'login'; passwordSet: boolean; notice?: string }
  | { kind: 'dashboard' };

/** Shows the login page until the robot confirms a session (spec NF-10). */
export function App() {
  const [view, setView] = useState<View>({ kind: 'checking' });

  const check = useCallback(async (notice?: string) => {
    try {
      const session = await getSession();
      setView(session.authenticated ? { kind: 'dashboard' } : { kind: 'login', passwordSet: session.passwordSet, notice });
    } catch (err) {
      setView({ kind: 'unreachable', message: err instanceof Error ? err.message : String(err) });
    }
  }, []);

  useEffect(() => {
    void check();
  }, [check]);

  const handleLogout = useCallback(async () => {
    await logout();
    await check();
  }, [check]);

  const handleSessionEnded = useCallback(() => {
    void check('Your session has ended. Please log in again.');
  }, [check]);

  switch (view.kind) {
    case 'checking':
      return <main className="login"><p>Connecting to the robot…</p></main>;
    case 'unreachable':
      return (
        <main className="login">
          <div className="login-card">
            <h1>Can't reach the robot</h1>
            <p>Check the WiFi link to the robot. ({view.message})</p>
            <button type="button" className="primary" onClick={() => void check()}>
              Try again
            </button>
          </div>
        </main>
      );
    case 'login':
      return (
        <>
          {view.notice && <p className="banner" role="status">{view.notice}</p>}
          <LoginPage passwordSet={view.passwordSet} onLoggedIn={() => void check()} />
        </>
      );
    case 'dashboard':
      return <Dashboard onLogout={() => void handleLogout()} onSessionEnded={handleSessionEnded} />;
  }
}
