import { useEffect, useState } from 'react'
import * as robotApi from '../api/robot'
import { useRobotStatus } from '../hooks/useRobotStatus'

type Command = 'emergency_stop' | 'release_estop'

export function EmergencyStopButtons() {
  const robotStatus = useRobotStatus()
  const [running, setRunning] = useState<Command | null>(null)
  const [message, setMessage] = useState('')
  const [displayEstop, setDisplayEstop] = useState(robotStatus.estop)

  useEffect(() => {
    if (running === null) setDisplayEstop(robotStatus.estop)
  }, [robotStatus.estop, running])

  const runCommand = async (command: Command) => {
    setRunning(command)
    setMessage('')
    try {
      const result =
        command === 'emergency_stop' ? await robotApi.emergencyStop() : await robotApi.releaseEstop()
      setMessage(result.message)
      setDisplayEstop(result.estop ?? command === 'emergency_stop')
    } catch (error) {
      setMessage(error instanceof Error ? error.message : '명령 실패')
    } finally {
      setRunning(null)
    }
  }

  const command: Command = displayEstop ? 'release_estop' : 'emergency_stop'
  const isRelease = command === 'release_estop'

  return (
    <div className="emergency-actions">
      <button
        type="button"
        className={`direct-action-btn ${isRelease ? 'emergency-release-btn' : 'emergency-stop-btn'}`}
        disabled={running !== null}
        onClick={() => void runCommand(command)}
      >
        {running === command ? '처리 중' : isRelease ? '비상정지 해제' : '비상정지'}
      </button>
      {message && <p className="emergency-action-message">{message}</p>}
    </div>
  )
}
