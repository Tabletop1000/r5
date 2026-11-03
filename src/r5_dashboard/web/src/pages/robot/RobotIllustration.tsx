/** Simple drawing of R5 for the Robot page (spec RB-3 allows a static image in v1). */
export function RobotIllustration() {
  return (
    <svg className="robot-illustration" viewBox="0 0 240 140" role="img" aria-label="R5 robot">
      <rect x="40" y="30" width="160" height="44" rx="6" className="ri-body" />
      <rect x="54" y="40" width="40" height="10" rx="3" className="ri-detail" />
      <rect x="62" y="74" width="10" height="22" className="ri-leg" />
      <rect x="168" y="74" width="10" height="22" className="ri-leg" />
      <circle cx="67" cy="104" r="26" className="ri-wheel" />
      <circle cx="67" cy="104" r="9" className="ri-hub" />
      <circle cx="173" cy="104" r="26" className="ri-wheel" />
      <circle cx="173" cy="104" r="9" className="ri-hub" />
    </svg>
  );
}
