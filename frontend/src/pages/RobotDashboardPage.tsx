import { useState } from 'react'

const JOINTS = [
  { name: 'joint_1', angle: '-' },
  { name: 'joint_2', angle: '-' },
  { name: 'joint_3', angle: '-' },
  { name: 'joint_4', angle: '-' },
  { name: 'joint_5', angle: '-' },
  { name: 'joint_6', angle: '-' },
  { name: 'gripper', angle: '-' },
]

const CONTROL_BUTTONS = ['홈 위치 이동', '일시정지', '재개', '정지', '그리퍼 열기', '그리퍼 닫기']

type SubTab = 'status' | 'control' | 'connection' | 'safety'

export function RobotDashboardPage() {
  const [subTab, setSubTab] = useState<SubTab>('status')

  return (
    <div>
      <nav className="tabs">
        <button
          type="button"
          className={subTab === 'status' ? 'tab tab-active' : 'tab'}
          onClick={() => setSubTab('status')}
        >
          상태
        </button>
        <button
          type="button"
          className={subTab === 'control' ? 'tab tab-active' : 'tab'}
          onClick={() => setSubTab('control')}
        >
          제어
        </button>
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

      {subTab === 'status' && (
        <div>
          <div className="summary-cards">
            <div className="stat-tile">
              <div className="stat-tile-label">
                <span className="status-dot" style={{ background: 'var(--status-critical)' }} />
                연결 상태
              </div>
              <div className="stat-tile-value">미연결</div>
            </div>
            <div className="stat-tile">
              <div className="stat-tile-label">모드</div>
              <div className="stat-tile-value">-</div>
            </div>
            <div className="stat-tile">
              <div className="stat-tile-label">현재 작업</div>
              <div className="stat-tile-value">대기</div>
            </div>
            <div className="stat-tile">
              <div className="stat-tile-label">컨트롤러</div>
              <div className="stat-tile-value">-</div>
            </div>
          </div>

          <table className="resource-table">
            <thead>
              <tr>
                <th>조인트</th>
                <th className="num-col">각도</th>
              </tr>
            </thead>
            <tbody>
              {JOINTS.map((joint) => (
                <tr key={joint.name}>
                  <td>{joint.name}</td>
                  <td className="num-col">{joint.angle}</td>
                </tr>
              ))}
            </tbody>
          </table>

          <p className="empty-state">ROS2/MoveIt 연동 후 실시간 로봇 상태가 여기에 표시됩니다.</p>
        </div>
      )}

      {subTab === 'control' && (
        <div>
          <div className="control-grid">
            {CONTROL_BUTTONS.map((label) => (
              <button type="button" className="control-btn" key={label} disabled>
                {label}
              </button>
            ))}
          </div>
          <p className="empty-state">ROS2 연동 후 버튼이 활성화됩니다.</p>
        </div>
      )}

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
