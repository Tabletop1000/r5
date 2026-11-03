interface Props {
  title: string;
  message: string;
}

/** Stand-in for pages that aren't built yet. */
export function PlaceholderPage({ title, message }: Props) {
  return (
    <section className="page">
      <h1>{title}</h1>
      <div className="placeholder">
        <p>{message}</p>
      </div>
    </section>
  );
}
