import { useState } from 'react'
import { useRobotStatus } from '../hooks/useRobotStatus'
import { useVoiceBridge } from '../hooks/useVoiceBridge'

const API_BASE = 'http://127.0.0.1:8000'

type DirectAction = {
  key: 'inspection' | 'delivery'
  label: string
  runningLabel: string
  errorLabel: string
  endpoint: string
  taskKey: string
}

const actions: DirectAction[] = [
  {
    key: 'inspection',
    label: '품질검사',
    runningLabel: '검사 진행 중...',
    errorLabel: '검사 노드 없음',
    endpoint: '/api/robot/run_inspection',
    taskKey: 'qc_running',
  },
  {
    key: 'delivery',
    label: '해머·드라이버 전달',
    runningLabel: '전달 진행 중...',
    errorLabel: '전달 실패',
    endpoint: '/api/robot/deliver_hammer_screwdriver',
    taskKey: 'tool_delivery',
  },
]

function BoltIcon() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M13 2 4 14h7l-1 8 9-12h-7l1-8Z"
        stroke="currentColor"
        strokeWidth="1.8"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  )
}

export function DirectRobotActionButtons() {
  const robotStatus = useRobotStatus(1000)
  const { hmiAlert } = useVoiceBridge()
  const [clickErrors, setClickErrors] = useState<Record<string, boolean>>({})

  const runAction = async (action: DirectAction) => {
    try {
      const res = await fetch(`${API_BASE}${action.endpoint}`, { method: 'POST' })
      const data = await res.json()
      if (!data.started) {
        setClickErrors((current) => ({ ...current, [action.key]: true }))
        window.setTimeout(() => {
          setClickErrors((current) => ({ ...current, [action.key]: false }))
        }, 3000)
      }
    } catch {
      setClickErrors((current) => ({ ...current, [action.key]: true }))
      window.setTimeout(() => {
        setClickErrors((current) => ({ ...current, [action.key]: false }))
      }, 3000)
    }
  }

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem', alignItems: 'flex-start' }}>
      <div style={{ display: 'flex', gap: '0.5rem', flexWrap: 'wrap' }}>
        {actions.map((action) => {
          const isRunning = robotStatus.task_key === action.taskKey
          const isError = clickErrors[action.key]
          const label = isRunning ? action.runningLabel : isError ? action.errorLabel : action.label

          return (
            <button
              type="button"
              key={action.key}
              className={`direct-action-btn${isRunning ? ' direct-action-btn-running' : ''}${
                isError ? ' direct-action-btn-error' : ''
              }`}
              onClick={() => void runAction(action)}
              disabled={isRunning || robotStatus.task_key !== 'idle'}
            >
              <BoltIcon />
              {label}
            </button>
          )
        })}
      </div>
      {hmiAlert && (
        <div style={{ 
          marginTop: '0.5rem', 
          padding: '0.75rem 1rem', 
          backgroundColor: '#ffebee', 
          color: '#c62828', 
          borderRadius: '8px',
          fontWeight: 'bold',
          fontSize: '0.9rem',
          border: '1px solid #ef9a9a',
          boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
          animation: 'fadeIn 0.3s'
        }}>
          ⚠️ {hmiAlert}
        </div>
      )}
    </div>
  )
}
