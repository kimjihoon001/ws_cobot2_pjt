import { useState } from 'react'

const API_BASE = 'http://127.0.0.1:8000'

function MicIcon() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M12 15a3 3 0 0 0 3-3V6a3 3 0 0 0-6 0v6a3 3 0 0 0 3 3Z"
        stroke="currentColor"
        strokeWidth="1.8"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M19 11a7 7 0 0 1-14 0M12 18v3"
        stroke="currentColor"
        strokeWidth="1.8"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  )
}

export function VoiceStartButton() {
  const [status, setStatus] = useState<'idle' | 'listening' | 'error'>('idle')

  const startListening = async () => {
    setStatus('listening')
    try {
      const res = await fetch(`${API_BASE}/api/robot/start_listen`, { method: 'POST' })
      const data = await res.json()
      if (!data.listening) setStatus('error')
      else window.setTimeout(() => setStatus('idle'), 4000)
    } catch {
      setStatus('error')
    }
  }

  const label =
    status === 'listening' ? '듣는 중... "hello rokey"' : status === 'error' ? '음성 노드 응답 없음' : '음성 시작'

  return (
    <div className="voice-btn-wrap">
      <button
        type="button"
        className={`voice-btn${status === 'listening' ? ' voice-btn-listening' : ''}${
          status === 'error' ? ' voice-btn-error' : ''
        }`}
        onClick={() => void startListening()}
      >
        <MicIcon />
        {label}
      </button>
      {status === 'error' && (
        <p className="voice-btn-hint">get_keyword 노드가 켜져 있는지 확인해주세요.</p>
      )}
    </div>
  )
}
