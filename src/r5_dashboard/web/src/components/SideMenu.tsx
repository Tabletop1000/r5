import { PAGES, hashForPage, type PageId } from '../routes';

interface Props {
  current: PageId;
  onLogout: () => void;
}

/** Left-hand menu (spec NAV-1). Becomes a bar along the bottom on narrow screens (spec NF-6). */
export function SideMenu({ current, onLogout }: Props) {
  return (
    <nav className="side-menu" aria-label="Pages">
      <div className="brand" aria-hidden="true">
        R5
      </div>
      <ul>
        {PAGES.map((page) => (
          <li key={page.id}>
            <a
              href={hashForPage(page.id)}
              className={page.id === current ? 'menu-item active' : 'menu-item'}
              aria-current={page.id === current ? 'page' : undefined}
            >
              {page.label}
            </a>
          </li>
        ))}
      </ul>
      <button type="button" className="menu-item logout" onClick={onLogout}>
        Log out
      </button>
    </nav>
  );
}
