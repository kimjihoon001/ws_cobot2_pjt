import { useVoiceBridge } from '../hooks/useVoiceBridge'
import { VoiceStartButton } from '../components/VoiceStartButton'

const JOINTS = [
  { name: 'joint_1', angle: '-' },
  { name: 'joint_2', angle: '-' },
  { name: 'joint_3', angle: '-' },
  { name: 'joint_4', angle: '-' },
  { name: 'joint_5', angle: '-' },
  { name: 'joint_6', angle: '-' },
  { name: 'gripper', angle: '-' },
]

const CONTROL_BUTTONS = [
  { label: '홈 위치 이동', variant: 'accent' },
  { label: '일시정지', variant: 'warning' },
  { label: '재개', variant: 'good' },
  { label: '정지', variant: 'critical-solid' },
  { label: '그리퍼 열기', variant: 'accent' },
  { label: '그리퍼 닫기', variant: 'accent' },
] as const

export function WorkSessionPage() {
  const { connected, pending, pendingRelease, respond, respondRelease } = useVoiceBridge()

  return (
    <div>
      <div className="task-status-row">
        <div className="stat-tile stat-tile-large">
          <div className="stat-tile-label">음성 명령</div>
          <VoiceStartButton />
        </div>
      </div>

      <div className="control-grid">
        {CONTROL_BUTTONS.map(({ label, variant }) => (
          <button
            type="button"
            className={`control-btn control-btn-${variant}`}
            key={label}
            disabled
          >
            {label}
          </button>
        ))}
      </div>
      <p className="empty-state">ROS2 연동 후 버튼이 활성화됩니다.</p>

      <div className="task-status-row">
        <div className="stat-tile stat-tile-large">
          {pending ? (
            <>
              <div className="stat-tile-label">확인 필요</div>
              <div className="stat-tile-value" style={{ fontSize: 22 }}>
                {pending.tools.map((t, i) => `${t} → ${pending.targets[i]}`).join(', ')}
              </div>
              <div style={{ display: 'flex', gap: 8, marginTop: 12 }}>
                <button type="button" className="primary-btn" onClick={() => respond(true)}>
                  확인
                </button>
                <button type="button" className="text-btn" onClick={() => respond(false)}>
                  아니오
                </button>
              </div>
            </>
          ) : (
            <>
              <div className="stat-tile-label">현재 작업</div>
              <div className="stat-tile-value">대기 중</div>
            </>
          )}
        </div>
        <div className="stat-tile-group">
          <div className="stat-tile-group-item">
            <div className="stat-tile-label">도구</div>
            <div className="stat-tile-value">{pending ? pending.tools.join(', ') : '-'}</div>
          </div>
          <div className="stat-tile-group-item">
            <div className="stat-tile-label">목적지</div>
            <div className="stat-tile-value">{pending ? pending.targets.join(', ') : '-'}</div>
          </div>
        </div>
      </div>
      <p className="empty-state">보이스 명령/ROS2 브릿지 연동 후 실제 작업 정보가 여기에 표시됩니다.</p>

      {pendingRelease && (
        <div className="task-status-row">
          <div className="stat-tile stat-tile-large">
            <div className="stat-tile-label">배송 확인</div>
            <div className="stat-tile-value" style={{ fontSize: 22 }}>
              도구를 받으셨나요? (
              {pendingRelease.tools.map((t, i) => `${t} → ${pendingRelease.targets[i]}`).join(', ')})
            </div>
            <div style={{ display: 'flex', gap: 8, marginTop: 12 }}>
              <button type="button" className="primary-btn" onClick={() => respondRelease(true)}>
                확인
              </button>
              <button type="button" className="text-btn" onClick={() => respondRelease(false)}>
                아니오
              </button>
            </div>
          </div>
        </div>
      )}

      <div className="task-status-row">
        <div className="stat-tile stat-tile-large">
          <div className="stat-tile-label">
            <span className="status-dot" style={{ background: 'var(--status-critical)' }} />
            연결 상태
          </div>
          <div className="stat-tile-value">{connected ? '연결됨' : '미연결'}</div>
        </div>
        <div className="stat-tile-group">
          <div className="stat-tile-group-item">
            <div className="stat-tile-label">모드</div>
            <div className="stat-tile-value">-</div>
          </div>
          <div className="stat-tile-group-item">
            <div className="stat-tile-label">컨트롤러</div>
            <div className="stat-tile-value">-</div>
          </div>
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
  )
}
