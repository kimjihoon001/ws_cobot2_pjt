import { useState } from 'react'
import * as robotApi from '../api/robot'
import { useVoiceBridge } from '../hooks/useVoiceBridge'

export function HmiAlertModal() {
  const { hmiAlert } = useVoiceBridge()
  const [running, setRunning] = useState<string | null>(null)
  const [dismissedAlertKey, setDismissedAlertKey] = useState('')

  const alertKey = hmiAlert ? String(hmiAlert.id) : ''
  const visibleAlert = hmiAlert && alertKey !== dismissedAlertKey ? hmiAlert : null
  const alertImageUrl = visibleAlert?.imageUrl?.startsWith('/')
    ? `http://127.0.0.1:8000${visibleAlert.imageUrl}`
    : visibleAlert?.imageUrl

  const runAction = async (command: string) => {
    if (command === 'dismiss') {
      setDismissedAlertKey(alertKey)
      return
    }
    setRunning(command)
    try {
      if (command === 'retry_jenga') await robotApi.runInspection()
      else if (command === 'retry_tool') await robotApi.retryPickTask()
      else await robotApi.cancelTask()
      setDismissedAlertKey(alertKey)
    } finally {
      setRunning(null)
    }
  }

  if (!visibleAlert) return null

  return (
    <div className="modal-backdrop hmi-alert-backdrop">
      <div className="modal hmi-alert-modal">
        <h2>{visibleAlert.title}</h2>
        <p>{visibleAlert.message}</p>
        {alertImageUrl && <img className="hmi-alert-image" src={alertImageUrl} alt="예외 발생 시점 화면" />}
        <div className="modal-actions">
          {visibleAlert.actions.map((action) => (
            <button
              type="button"
              key={action.command}
              className={action.variant === 'danger' ? 'control-btn control-btn-critical-solid' : 'primary-btn'}
              disabled={running !== null}
              onClick={() => void runAction(action.command)}
            >
              {running === action.command ? '처리 중' : action.label}
            </button>
          ))}
        </div>
      </div>
    </div>
  )
}
