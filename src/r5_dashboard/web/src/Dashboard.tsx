import { useEffect, useState } from 'react';
import { getSession } from './api';
import { SideMenu } from './components/SideMenu';
import { StatusStrip } from './components/StatusStrip';
import { PlaceholderPage } from './pages/PlaceholderPage';
import { SprayPage } from './pages/SprayPage';
import { RobotConnection, defaultRosUrl } from './ros/connection';
import { RobotContext } from './ros/RobotContext';
import { pageFromHash, type PageId } from './routes';

interface Props {
  onLogout: () => void;
  onSessionEnded: () => void;
}

/** The logged-in app: menu, status strip and one page at a time (spec NAV-1, NAV-2, NAV-3). */
export function Dashboard({ onLogout, onSessionEnded }: Props) {
  const page = useHashPage();
  const [robot, setRobot] = useState<RobotConnection | null>(null);

  // Made inside the effect so each mount owns exactly one connection and closes it on unmount.
  useEffect(() => {
    const connection = new RobotConnection({
      url: defaultRosUrl(),
      isLoggedIn: async () => (await getSession()).authenticated,
      onLoggedOut: onSessionEnded,
    });
    connection.start();
    setRobot(connection);
    return () => connection.stop();
  }, [onSessionEnded]);

  if (!robot) return null;

  return (
    <RobotContext.Provider value={robot}>
      <div className="shell">
        <SideMenu current={page} onLogout={onLogout} />
        <div className="main">
          <StatusStrip />
          <main className="content">{renderPage(page)}</main>
        </div>
      </div>
    </RobotContext.Provider>
  );
}

function renderPage(page: PageId) {
  switch (page) {
    case 'fields':
      return <PlaceholderPage title="Manage Fields" message="Field management is coming in a later update." />;
    case 'spray':
      return <SprayPage />;
    case 'mow':
      return <PlaceholderPage title="Mow" message="Coming soon. Mowing is not available yet." />;
    case 'robot':
      return <PlaceholderPage title="Robot" message="Robot status and tests are coming in the next update." />;
  }
}

function useHashPage(): PageId {
  const [page, setPage] = useState(() => pageFromHash(window.location.hash));
  useEffect(() => {
    const onChange = () => setPage(pageFromHash(window.location.hash));
    window.addEventListener('hashchange', onChange);
    return () => window.removeEventListener('hashchange', onChange);
  }, []);
  return page;
}
