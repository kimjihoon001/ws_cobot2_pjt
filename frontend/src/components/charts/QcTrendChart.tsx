import { useState } from 'react'
import type { QcTrendPoint } from '../../types/qc'

const CHART_HEIGHT = 140

export function QcTrendChart({ data }: { data: QcTrendPoint[] }) {
  const [hover, setHover] = useState<number | null>(null)

  const hasData = data.some((d) => d.passed + d.failed > 0)

  if (!hasData) {
    return (
      <div className="chart-card">
        <h4 className="chart-title">최근 {data.length}일 검사 현황</h4>
        <p className="empty-state">아직 검사 이력이 없습니다.</p>
      </div>
    )
  }

  const max = Math.max(...data.map((d) => d.passed + d.failed), 1)

  return (
    <div className="chart-card">
      <div className="chart-head">
        <h4 className="chart-title">최근 {data.length}일 검사 현황</h4>
        <div className="chart-legend">
          <span>
            <i style={{ background: 'var(--status-good)' }} />
            PASS
          </span>
          <span>
            <i style={{ background: 'var(--status-critical)' }} />
            FAIL
          </span>
        </div>
      </div>

      <div className="bar-chart-plot" style={{ height: CHART_HEIGHT }}>
        {data.map((d, i) => {
          const total = d.passed + d.failed
          const passH = (d.passed / max) * CHART_HEIGHT
          const failH = (d.failed / max) * CHART_HEIGHT
          const failOnTop = d.failed > 0
          return (
            <div
              className="bar-col"
              key={d.date}
              tabIndex={0}
              onMouseEnter={() => setHover(i)}
              onMouseLeave={() => setHover(null)}
              onFocus={() => setHover(i)}
              onBlur={() => setHover(null)}
            >
              {(hover === i || total > 0) && <span className="bar-value-label">{total}</span>}
              <div className="stacked-bar">
                {failH > 0 && (
                  <div
                    className="bar"
                    style={{
                      height: failH,
                      background: 'var(--status-critical)',
                      borderRadius: '4px 4px 0 0',
                    }}
                  />
                )}
                {failH > 0 && passH > 0 && <div className="bar-gap" />}
                {passH > 0 && (
                  <div
                    className="bar"
                    style={{
                      height: passH,
                      background: 'var(--status-good)',
                      borderRadius: failOnTop ? 0 : '4px 4px 0 0',
                    }}
                  />
                )}
              </div>
              {hover === i && (
                <div className="chart-tooltip">
                  <div>
                    <strong>{d.passed}</strong> PASS
                  </div>
                  <div>
                    <strong>{d.failed}</strong> FAIL
                  </div>
                </div>
              )}
            </div>
          )
        })}
      </div>
      <div className="bar-chart-labels">
        {data.map((d) => (
          <span key={d.date}>{d.date.slice(5)}</span>
        ))}
      </div>
    </div>
  )
}
