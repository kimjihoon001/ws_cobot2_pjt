import type { Summary } from '../types/resource'

export function SummaryCards({ summary }: { summary: Summary }) {
  const tiles: { label: string; value: number; color?: string }[] = [
    { label: '전체 자원', value: summary.total },
    { label: '정상', value: summary.normal, color: 'var(--status-good)' },
    { label: '부족', value: summary.low, color: 'var(--status-warning)' },
    { label: '품절', value: summary.out, color: 'var(--status-critical)' },
  ]

  return (
    <div className="summary-cards">
      {tiles.map((tile) => (
        <div className="stat-tile" key={tile.label}>
          <div className="stat-tile-label">
            {tile.color && <span className="status-dot" style={{ background: tile.color }} />}
            {tile.label}
          </div>
          <div className="stat-tile-value">{tile.value.toLocaleString()}</div>
        </div>
      ))}
    </div>
  )
}
