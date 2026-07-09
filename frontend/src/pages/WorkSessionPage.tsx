import { useState } from 'react'
import { useVoiceBridge } from '../hooks/useVoiceBridge'
import { useRobotStatus } from '../hooks/useRobotStatus'
import { VoiceStartButton } from '../components/VoiceStartButton'
import * as robotApi from '../api/robot'

const JOINTS = [
  'joint_1',
  'joint_2',
  'joint_3',
  'joint_4',
  'joint_5',
  'joint_6',
  'gripper',
]

const CONTROL_BUTTONS = [
  { label: '홈 위치 이동', variant: 'accent', action: undefined },
  { label: '일시정지', variant: 'warning', action: undefined },
  { label: '비상해제', variant: 'good', action: 'release_estop' },
  { label: '비상정지', variant: 'critical-solid', action: 'emergency_stop' },
  { label: '그리퍼 열기', variant: 'accent', action: undefined },
  { label: '그리퍼 닫기', variant: 'accent', action: undefined },
] as const

export function WorkSessionPage() {
  const { connected, pending, pendingRelease, respond, respondRelease } = useVoiceBridge()
  const robotStatus = useRobotStatus()
  const [robotCommandMessage, setRobotCommandMessage] = useState('')
  const [robotCommandRunning, setRobotCommandRunning] = useState<string | null>(null)
  const robotStatusColor = robotStatus.connected ? 'var(--status-good)' : 'var(--status-critical)'
  const formatJointValue = (name: string) => {
    const value = robotStatus.joints[name]
    if (value === null || value === undefined) return '-'
    const unit = robotStatus.joint_units[name] ?? ''
    return `${value.toFixed(name === 'gripper' ? 1 : 2)} ${unit}`.trim()
  }
  const runControlAction = async (action?: string) => {
    if (!action) return
    setRobotCommandRunning(action)
    setRobotCommandMessage('')
    try {
      const result =
        action === 'emergency_stop' ? await robotApi.emergencyStop() : await robotApi.releaseEstop()
      setRobotCommandMessage(result.message)
    } catch (error) {
      setRobotCommandMessage(error instanceof Error ? error.message : '명령 실패')
    } finally {
      setRobotCommandRunning(null)
    }
  }

  return (
    <div>
      <div className="task-status-row">
        <div className="stat-tile stat-tile-large">
          <div className="stat-tile-label">음성 명령</div>
          <VoiceStartButton />
        </div>
      </div>

      <div className="control-grid">
        {CONTROL_BUTTONS.map(({ label, variant, action }) => (
          <button
            type="button"
            className={`control-btn control-btn-${variant}`}
            key={label}
            disabled={!action || robotCommandRunning !== null || (action === 'release_estop' && !robotStatus.estop)}
            onClick={() => void runControlAction(action)}
          >
            {robotCommandRunning === action ? '처리 중' : label}
          </button>
        ))}
      </div>
      <p className="empty-state">
        {robotCommandMessage || '비상정지는 DSR move_stop과 servo_off를 동시에 요청합니다.'}
      </p>

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
              <div className="stat-tile-value" style={{ fontSize: 38 }}>
                {robotStatus.current_task}
              </div>
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
            <span className="status-dot" style={{ background: robotStatusColor }} />
            연결 상태
          </div>
          <div className="stat-tile-value">{robotStatus.connected ? '연결됨' : '미연결'}</div>
          <div className="stat-tile-label" style={{ marginTop: 8 }}>
            HMI 브릿지: {connected ? '연결됨' : '미연결'}
          </div>
        </div>
        <div className="stat-tile-group">
          <div className="stat-tile-group-item">
            <div className="stat-tile-label">모드</div>
            <div className="stat-tile-value">{robotStatus.mode}</div>
          </div>
          <div className="stat-tile-group-item">
            <div className="stat-tile-label">컨트롤러</div>
            <div className="stat-tile-value">{robotStatus.controller}</div>
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
          {JOINTS.map((jointName) => (
            <tr key={jointName}>
              <td>{jointName}</td>
              <td className="num-col">{formatJointValue(jointName)}</td>
            </tr>
          ))}
        </tbody>
      </table>

      <p className="empty-state">ROS2/MoveIt 연동 후 실시간 로봇 상태가 여기에 표시됩니다.</p>
    </div>
  )
}
