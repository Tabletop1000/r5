// The pages in the side menu (spec NAV-1). Pages use hash addresses (#/robot) so a refresh keeps
// the current page and the server only ever has to serve index.html.

export type PageId = 'fields' | 'spray' | 'mow' | 'robot';

export const PAGES: { id: PageId; label: string }[] = [
  { id: 'fields', label: 'Manage Fields' },
  { id: 'spray', label: 'Spray' },
  { id: 'mow', label: 'Mow' },
  { id: 'robot', label: 'Robot' },
];

export const DEFAULT_PAGE: PageId = 'fields';

export function pageFromHash(hash: string): PageId {
  const id = hash.replace(/^#\/?/, '').split(/[/?]/)[0];
  return PAGES.some((p) => p.id === id) ? (id as PageId) : DEFAULT_PAGE;
}

export function hashForPage(id: PageId): string {
  return `#/${id}`;
}
