export function QcPage() {
  const history: { id: string; time: string; product: string; result: string; location: string }[] = []

  return (
    <div>
      <div className="summary-cards">
        <div className="stat-tile">
          <div className="stat-tile-label">검사 대기</div>
          <div className="stat-tile-value">0</div>
        </div>
        <div className="stat-tile">
          <div className="stat-tile-label">
            <span className="status-dot" style={{ background: 'var(--status-good)' }} />
            PASS
          </div>
          <div className="stat-tile-value">0</div>
        </div>
        <div className="stat-tile">
          <div className="stat-tile-label">
            <span className="status-dot" style={{ background: 'var(--status-critical)' }} />
            FAIL
          </div>
          <div className="stat-tile-value">0</div>
        </div>
        <div className="stat-tile">
          <div className="stat-tile-label">불량률</div>
          <div className="stat-tile-value">-</div>
        </div>
      </div>

      <div className="qc-image-placeholder">검사 이미지 없음</div>

      <table className="resource-table">
        <thead>
          <tr>
            <th>일시</th>
            <th>제품</th>
            <th>결과</th>
            <th>불량 위치</th>
          </tr>
        </thead>
        <tbody>
          {history.length === 0 ? (
            <tr>
              <td colSpan={4} className="empty-state">
                아직 검사 이력이 없습니다.
              </td>
            </tr>
          ) : (
            history.map((row) => (
              <tr key={row.id}>
                <td>{row.time}</td>
                <td>{row.product}</td>
                <td>{row.result}</td>
                <td>{row.location}</td>
              </tr>
            ))
          )}
        </tbody>
      </table>

      <p className="empty-state">YOLO 불량 위치 인식 연동 후, 검사 이미지 위에 불량 위치가 표시됩니다.</p>
    </div>
  )
}
