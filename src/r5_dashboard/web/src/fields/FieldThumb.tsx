import { LocalProjection, type FieldDraft } from './geo';

const W = 120;
const H = 84;
const PAD = 6;

/** Outline of a field for its card (spec FM-1). */
export function FieldThumb({ field }: { field: Pick<FieldDraft, 'boundary' | 'keepOut'> }) {
  if (field.boundary.length < 3) return <svg className="field-thumb" viewBox={`0 0 ${W} ${H}`} aria-hidden="true" />;
  const proj = LocalProjection.around(field.boundary);
  const outer = field.boundary.map((p) => proj.toXY(p));
  const xs = outer.map((p) => p[0]);
  const ys = outer.map((p) => p[1]);
  const minX = Math.min(...xs);
  const maxY = Math.max(...ys);
  const scale = Math.min((W - 2 * PAD) / (Math.max(...xs) - minX || 1), (H - 2 * PAD) / (maxY - Math.min(...ys) || 1));
  const offX = (W - (Math.max(...xs) - minX) * scale) / 2;
  const offY = (H - (maxY - Math.min(...ys)) * scale) / 2;
  const path = (ring: [number, number][]) =>
    ring.map(([x, y], i) => `${i ? 'L' : 'M'}${(offX + (x - minX) * scale).toFixed(1)},${(offY + (maxY - y) * scale).toFixed(1)}`).join('') + 'Z';
  return (
    <svg className="field-thumb" viewBox={`0 0 ${W} ${H}`} aria-hidden="true">
      <path d={path(outer)} className="thumb-field" />
      {field.keepOut.map((z, i) => (
        <path key={i} d={path(z.map((p) => proj.toXY(p)))} className="thumb-keepout" />
      ))}
    </svg>
  );
}
