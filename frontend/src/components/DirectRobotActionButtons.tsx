import { useState } from 'react'

const API_BASE = 'http://127.0.0.1:8000'

type ActionStatus = 'idle' | 'running' | 'error'

type DirectAction = {
  key: 'inspection' | 'delivery'
  label: string
  runningLabel: string
  errorLabel: string
  endpoint: string
}

const actions: DirectAction[] = [
  {
    key: 'inspection',
    label: '품질검사',
    runningLabel: '검사 시작됨',
    errorLabel: '검사 노드 없음',
    endpoint: '/api/robot/run_inspection',
  },
  {
    key: 'delivery',
    label: '해머·드라이버 전달',
    runningLabel: '전달 시작됨',
    errorLabel: '전달 실패',
    endpoint: '/api/robot/deliver_hammer_screwdriver',
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
  const [statuses, setStatuses] = useState<Record<DirectAction['key'], ActionStatus>>({
    inspection: 'idle',
    delivery: 'idle',
  })

  const runAction = async (action: DirectAction) => {
    setStatuses((current) => ({ ...current, [action.key]: 'running' }))
    try {
      const res = await fetch(`${API_BASE}${action.endpoint}`, { method: 'POST' })
      const data = await res.json()
      if (!data.started) {
        setStatuses((current) => ({ ...current, [action.key]: 'error' }))
        return
      }
      window.setTimeout(() => {
        setStatuses((current) => ({ ...current, [action.key]: 'idle' }))
      }, 2500)
    } catch {
      setStatuses((current) => ({ ...current, [action.key]: 'error' }))
    }
  }

  return (
    <>
      {actions.map((action) => {
        const status = statuses[action.key]
        const label =
          status === 'running' ? action.runningLabel : status === 'error' ? action.errorLabel : action.label

        return (
          <button
            type="button"
            key={action.key}
            className={`direct-action-btn${status === 'running' ? ' direct-action-btn-running' : ''}${
              status === 'error' ? ' direct-action-btn-error' : ''
            }`}
            onClick={() => void runAction(action)}
          >
            <BoltIcon />
            {label}
          </button>
        )
      })}
    </>
  )
}
