import { useState } from 'react';

type Mode = 'area' | 'spot';

/** Spray page with Area Spray and Spot Spray modes (spec NAV-5). Content comes in a later step. */
export function SprayPage() {
  const [mode, setMode] = useState<Mode>('area');
  return (
    <section className="page">
      <h1>Spray</h1>
      <div className="tabs" role="tablist" aria-label="Spray mode">
        <Tab id="area" label="Area Spray" mode={mode} onSelect={setMode} />
        <Tab id="spot" label="Spot Spray" mode={mode} onSelect={setMode} />
      </div>
      <div className="placeholder" role="tabpanel">
        <p>
          {mode === 'area'
            ? 'Area spray setup is coming in a later update.'
            : 'Coming soon. Spot spraying is not available yet.'}
        </p>
      </div>
    </section>
  );
}

function Tab({ id, label, mode, onSelect }: { id: Mode; label: string; mode: Mode; onSelect: (m: Mode) => void }) {
  return (
    <button type="button" role="tab" aria-selected={mode === id} className={mode === id ? 'tab active' : 'tab'} onClick={() => onSelect(id)}>
      {label}
    </button>
  );
}
