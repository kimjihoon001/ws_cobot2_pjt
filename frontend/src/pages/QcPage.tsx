import { useCallback, useEffect, useState } from 'react'
import * as api from '../api/qc'
import type { Inspection, InspectionResult, QcSummary, QcTrendPoint } from '../types/qc'
import { QcTrendChart } from '../components/charts/QcTrendChart'
import { Pagination } from '../components/Pagination'
import { SortableTh } from '../components/SortableTh'
import { downloadCsv } from '../utils/csv'
import { sortBy } from '../utils/sort'
import type { SortDir } from '../utils/sort'

type InspectionSortKey = 'created_at' | 'product' | 'result'

const PAGE_SIZE = 10

export function QcPage() {
  const [inspections, setInspections] = useState<Inspection[]>([])
  const [summary, setSummary] = useState<QcSummary | null>(null)
  const [trend, setTrend] = useState<QcTrendPoint[]>([])
  const [result, setResult] = useState<InspectionResult | ''>('')
  const [product, setProduct] = useState('')
  const [page, setPage] = useState(1)
  const [sortKey, setSortKey] = useState<InspectionSortKey>('created_at')
  const [sortDir, setSortDir] = useState<SortDir>('desc')
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  const refresh = useCallback(async () => {
    try {
      const [list, sum, tr] = await Promise.all([
        api.listInspections({ result: result || undefined, product: product || undefined }),
        api.getQcSummary(),
        api.getQcTrend(7),
      ])
      setInspections(list)
      setSummary(sum)
      setTrend(tr)
      setError(null)
    } catch (err) {
      setError(err instanceof Error ? err.message : '데이터를 불러오지 못했습니다.')
    } finally {
      setLoading(false)
    }
  }, [result, product])

  useEffect(() => {
    refresh()
  }, [refresh])

  useEffect(() => {
    setPage(1)
  }, [result, product, sortKey, sortDir])

  function handleSort(key: InspectionSortKey) {
    if (key === sortKey) {
      setSortDir((d) => (d === 'asc' ? 'desc' : 'asc'))
    } else {
      setSortKey(key)
      setSortDir('asc')
    }
  }

  function handleExport() {
    downloadCsv(
      `qc_inspections_${new Date().toISOString().slice(0, 10)}.csv`,
      ['일시', '제품', '결과', '불량 위치'],
      inspections.map((row) => [
        new Date(row.created_at).toLocaleString(),
        row.product,
        row.result === 'pass' ? 'PASS' : 'FAIL',
        row.defect_location ?? '',
      ]),
    )
  }

  const sortedInspections = sortBy(inspections, sortKey, sortDir)
  const pagedInspections = sortedInspections.slice((page - 1) * PAGE_SIZE, page * PAGE_SIZE)

  return (
    <div>
      {error && <div className="banner-error">{error}</div>}

      <div className="summary-cards">
        <div className="stat-tile">
          <div className="stat-tile-label">전체 검사</div>
          <div className="stat-tile-value">{summary ? summary.total : '-'}</div>
        </div>
        <div className="stat-tile">
          <div className="stat-tile-label">
            <span className="status-dot" style={{ background: 'var(--status-good)' }} />
            PASS
          </div>
          <div className="stat-tile-value">{summary ? summary.passed : '-'}</div>
        </div>
        <div className="stat-tile">
          <div className="stat-tile-label">
            <span className="status-dot" style={{ background: 'var(--status-critical)' }} />
            FAIL
          </div>
          <div className="stat-tile-value">{summary ? summary.failed : '-'}</div>
        </div>
        <div className="stat-tile">
          <div className="stat-tile-label">불량률</div>
          <div className="stat-tile-value">
            {summary && summary.defect_rate !== null ? `${summary.defect_rate}%` : '-'}
          </div>
        </div>
      </div>

      <QcTrendChart data={trend} />

      <div className="search-bar">
        <input
          className="search-input"
          placeholder="제품명으로 검색"
          value={product}
          onChange={(e) => setProduct(e.target.value)}
        />
        <select value={result} onChange={(e) => setResult(e.target.value as InspectionResult | '')}>
          <option value="">전체 결과</option>
          <option value="pass">PASS</option>
          <option value="fail">FAIL</option>
        </select>
        {inspections.length > 0 && (
          <button type="button" className="text-btn" onClick={handleExport}>
            CSV 내보내기
          </button>
        )}
      </div>

      {loading ? (
        <p className="empty-state">불러오는 중...</p>
      ) : (
        <>
          <table className="resource-table">
            <thead>
              <tr>
                <SortableTh label="일시" sortKey="created_at" activeKey={sortKey} dir={sortDir} onSort={handleSort} />
                <SortableTh label="제품" sortKey="product" activeKey={sortKey} dir={sortDir} onSort={handleSort} />
                <SortableTh label="결과" sortKey="result" activeKey={sortKey} dir={sortDir} onSort={handleSort} />
                <th>불량 위치</th>
              </tr>
            </thead>
            <tbody>
              {pagedInspections.length === 0 ? (
                <tr>
                  <td colSpan={4} className="empty-state">
                    아직 검사 이력이 없습니다. YOLO 연동 후 이곳에 자동으로 기록됩니다.
                  </td>
                </tr>
              ) : (
                pagedInspections.map((row) => {
                  const color =
                    row.result === 'pass' ? 'var(--status-good)' : 'var(--status-critical)'
                  return (
                    <tr key={row.id}>
                      <td>{new Date(row.created_at).toLocaleString()}</td>
                      <td>{row.product}</td>
                      <td>
                        <span
                          className="status-badge"
                          style={{
                            background: `color-mix(in srgb, ${color} 14%, transparent)`,
                            borderColor: `color-mix(in srgb, ${color} 40%, transparent)`,
                          }}
                        >
                          <span className="status-dot" style={{ background: color }} />
                          {row.result === 'pass' ? 'PASS' : 'FAIL'}
                        </span>
                      </td>
                      <td>{row.defect_location ?? '-'}</td>
                    </tr>
                  )
                })
              )}
            </tbody>
          </table>
          <Pagination
            page={page}
            pageSize={PAGE_SIZE}
            total={inspections.length}
            onPageChange={setPage}
          />
        </>
      )}

      <div className="qc-image-placeholder">검사 이미지 없음</div>
      <p className="empty-state">YOLO 불량 위치 인식 연동 후, 검사 이미지 위에 불량 위치가 표시됩니다.</p>
    </div>
  )
}
