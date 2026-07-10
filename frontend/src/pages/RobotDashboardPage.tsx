import { useState } from 'react'
import { useRobotStatus } from '../hooks/useRobotStatus'

type SubTab = 'connection' | 'safety'

const CHECK_LABELS = {
  dsr: 'Doosan 제어기',
  moveit: 'MoveIt',
  conveyor: '컨베이어벨트',
  jenga_inspector: '품질검사 노드',
  tool_pick: '공구 전달 노드',
  voice: '음성 노드',
  hand: '손 감지 노드',
} as const

export function RobotDashboardPage() {
  const [subTab, setSubTab] = useState<SubTab>('connection')
  const robotStatus = useRobotStatus()
  const statusColor = robotStatus.connected ? 'var(--status-good)' : 'var(--status-critical)'

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
          <div className="summary-cards">
            <div className="stat-tile">
              <div className="stat-tile-label">
                <span className="status-dot" style={{ background: statusColor }} />
                연결 상태
              </div>
              <div className="stat-tile-value">{robotStatus.connected ? '연결됨' : '미연결'}</div>
            </div>
            <div className="stat-tile">
              <div className="stat-tile-label">현재 작업</div>
              <div className="stat-tile-value">{robotStatus.current_task}</div>
            </div>
            <div className="stat-tile">
              <div className="stat-tile-label">컨트롤러</div>
              <div className="stat-tile-value">{robotStatus.controller}</div>
            </div>
          </div>

          <div className="field-grid">
            <label>
              로봇 IP
              <input value="192.168.1.100" disabled />
            </label>
            <label>
              포트
              <input value="ROS2" disabled />
            </label>
            <label>
              모드
              <input value={robotStatus.mode} disabled />
            </label>
          </div>

          <div className="resource-table-wrap">
            <table className="resource-table">
              <thead>
                <tr>
                  <th>항목</th>
                  <th>상태</th>
                </tr>
              </thead>
              <tbody>
                {Object.entries(CHECK_LABELS).map(([key, label]) => {
                  const ok = robotStatus.checks[key as keyof typeof robotStatus.checks]
                  return (
                    <tr key={key}>
                      <td>{label}</td>
                      <td>
                        <span className="status-badge">
                          <span
                            className="status-dot"
                            style={{ background: ok ? 'var(--status-good)' : 'var(--status-critical)' }}
                          />
                          {ok ? '연결됨' : '미연결'}
                        </span>
                      </td>
                    </tr>
                  )
                })}
              </tbody>
            </table>
          </div>
        </div>
      )}

      {subTab === 'safety' && (
        <div>
          <div className="summary-cards">
            <div className="stat-tile">
              <div className="stat-tile-label">
                <span className="status-dot" style={{ background: 'var(--status-good)' }} />
                안전 모니터
              </div>
              <div className="stat-tile-value">{robotStatus.checks.hand ? '연동' : '미연동'}</div>
            </div>
            <div className="stat-tile">
              <div className="stat-tile-label">MoveIt</div>
              <div className="stat-tile-value">{robotStatus.checks.moveit ? '정상' : '미연결'}</div>
            </div>
            <div className="stat-tile">
              <div className="stat-tile-label">사람 감지</div>
              <div className="stat-tile-value">{robotStatus.checks.hand ? '활성' : '대기'}</div>
            </div>
          </div>
        </div>
      )}
    </div>
  )
}
