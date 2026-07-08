import { useState } from 'react'

const CHART_HEIGHT = 140

export function CategoryBarChart({
  title,
  data,
}: {
  title: string
  data: { label: string; value: number }[]
}) {
  const [hover, setHover] = useState<number | null>(null)

  if (data.length === 0) {
    return (
      <div className="chart-card">
        <h4 className="chart-title">{title}</h4>
        <p className="empty-state">표시할 데이터가 없습니다.</p>
      </div>
    )
  }

  const max = Math.max(...data.map((d) => d.value), 1)
  const maxIndex = data.reduce((best, d, i) => (d.value > data[best].value ? i : best), 0)

  return (
    <div className="chart-card">
      <h4 className="chart-title">{title}</h4>
      <div className="bar-chart-plot" style={{ height: CHART_HEIGHT }}>
        {data.map((d, i) => (
          <div
            className="bar-col"
            key={d.label}
            tabIndex={0}
            onMouseEnter={() => setHover(i)}
            onMouseLeave={() => setHover(null)}
            onFocus={() => setHover(i)}
            onBlur={() => setHover(null)}
          >
            {(hover === i || i === maxIndex) && (
              <span className="bar-value-label">{d.value.toLocaleString()}</span>
            )}
            <div
              className="bar"
              style={{
                height: Math.max((d.value / max) * CHART_HEIGHT, 2),
                background: 'var(--accent)',
                borderRadius: '4px 4px 0 0',
              }}
            />
            {hover === i && (
              <div className="chart-tooltip">
                <strong>{d.value.toLocaleString()}</strong>
                <span>{d.label}</span>
              </div>
            )}
          </div>
        ))}
      </div>
      <div className="bar-chart-labels">
        {data.map((d) => (
          <span key={d.label}>{d.label}</span>
        ))}
      </div>
    </div>
  )
}
