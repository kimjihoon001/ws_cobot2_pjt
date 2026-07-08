import { useState } from 'react'

type SubTab = 'connection' | 'safety'

export function RobotDashboardPage() {
  const [subTab, setSubTab] = useState<SubTab>('connection')

  return (
    <div>
      <nav className="tabs">
        <button
          type="button"
          className={subTab === 'connection' ? 'tab tab-active' : 'tab'}
          onClick={() => setSubTab('connection')}
        >
          연결
        </button>
        <button
          type="button"
          className={subTab === 'safety' ? 'tab tab-active' : 'tab'}
          onClick={() => setSubTab('safety')}
        >
          안전관리
        </button>
      </nav>

      {subTab === 'connection' && (
        <div>
          <div className="field-grid">
            <label>
              로봇 IP
              <input value="" placeholder="예: 192.168.1.100" disabled />
            </label>
            <label>
              포트
              <input value="" placeholder="예: 12345" disabled />
            </label>
            <label>
              모드
              <input value="" placeholder="virtual / real" disabled />
            </label>
          </div>
          <button type="button" className="primary-btn" disabled>
            연결
          </button>
          <p className="empty-state">ROS2 브릿지 연동 후 로봇/그리퍼 연결을 관리할 수 있습니다.</p>
        </div>
      )}

      {subTab === 'safety' && (
        <div>
          <div className="summary-cards">
            <div className="stat-tile">
              <div className="stat-tile-label">
                <span className="status-dot" style={{ background: 'var(--status-critical)' }} />
                비상정지
              </div>
              <div className="stat-tile-value">-</div>
            </div>
            <div className="stat-tile">
              <div className="stat-tile-label">협동 안전 존</div>
              <div className="stat-tile-value">-</div>
            </div>
            <div className="stat-tile">
              <div className="stat-tile-label">사람 감지</div>
              <div className="stat-tile-value">-</div>
            </div>
          </div>
          <p className="empty-state">
            손 회피(사람 감지) 로직 연동 후 안전 상태와 정지 이력이 여기에 표시됩니다.
          </p>
        </div>
      )}
    </div>
  )
}
