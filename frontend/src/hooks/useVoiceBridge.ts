import { useCallback, useEffect, useRef, useState } from 'react'

export interface PendingConfirm {
  tools: string[]
  targets: string[]
}

const API_BASE = 'http://127.0.0.1:8000'

export function useVoiceBridge(url = 'ws://127.0.0.1:8000/ws') {
  const wsRef = useRef<WebSocket | null>(null)
  const [pending, setPending] = useState<PendingConfirm | null>(null)
  const [connected, setConnected] = useState(false)

  useEffect(() => {
    let stopped = false
    let retryTimer: number | undefined

    const connect = () => {
      const ws = new WebSocket(url)
      wsRef.current = ws

      ws.onopen = () => setConnected(true)

      ws.onmessage = (ev) => {
        try {
          const msg = JSON.parse(ev.data)
          if (msg.type === 'confirm_request') {
            setPending({ tools: msg.tools, targets: msg.targets })
          }
        } catch {
          // JSON이 아닌 메시지는 무시
        }
      }

      ws.onclose = () => {
        setConnected(false)
        if (!stopped) {
          retryTimer = window.setTimeout(connect, 1000)
        }
      }
    }

    connect()

    return () => {
      stopped = true
      if (retryTimer) window.clearTimeout(retryTimer)
      wsRef.current?.close()
    }
  }, [url])

  useEffect(() => {
    const poll = window.setInterval(async () => {
      try {
        const res = await fetch(`${API_BASE}/api/voice/pending`)
        const data = await res.json()
        if (data.pending?.tools && data.pending?.targets) {
          setPending({ tools: data.pending.tools, targets: data.pending.targets })
        }
      } catch {
        // WebSocket 재연결 루프가 따로 있으므로 폴링 실패는 조용히 무시
      }
    }, 1000)

    return () => window.clearInterval(poll)
  }, [])

  const respond = useCallback((confirmed: boolean) => {
    wsRef.current?.send(JSON.stringify({ cmd: 'confirm_response', confirmed }))
    setPending(null)
  }, [])

  return { connected, pending, respond }
}
