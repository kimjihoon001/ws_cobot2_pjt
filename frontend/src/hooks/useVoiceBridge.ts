import { useCallback, useEffect, useRef, useState } from 'react'

export interface PendingConfirm {
  id: number
  tools: string[]
  targets: string[]
}

const API_BASE = 'http://127.0.0.1:8000'

export function useVoiceBridge(url = 'ws://127.0.0.1:8000/ws') {
  const wsRef = useRef<WebSocket | null>(null)
  const [pending, setPending] = useState<PendingConfirm | null>(null)
  const [pendingRelease, setPendingRelease] = useState<PendingConfirm | null>(null)
  const [connected, setConnected] = useState(false)

  // WS는 폴링 주기(1초)보다 빠르게 새 요청이 생겼다는 걸 알아채는 용도로만 사용 —
  // 실제 정답 소스와 응답 제출은 전부 DB 기반 REST(/api/voice/requests/*)로 처리한다.
  useEffect(() => {
    let stopped = false
    let retryTimer: number | undefined

    const connect = () => {
      const ws = new WebSocket(url)
      wsRef.current = ws

      ws.onopen = () => setConnected(true)
      ws.onmessage = () => {
        // 페이로드 내용과 무관하게, 뭔가 새로 생겼다는 신호로만 쓰고 폴링에서 다시 읽는다.
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
        const res = await fetch(`${API_BASE}/api/voice/requests/pending`)
        const data = await res.json()
        const requests: Array<PendingConfirm & { kind: string }> = data.requests ?? []
        const toolConfirm = requests.find((r) => r.kind === 'tool_confirm') ?? null
        const releaseConfirm = requests.find((r) => r.kind === 'release_confirm') ?? null
        setPending(toolConfirm ? { id: toolConfirm.id, tools: toolConfirm.tools, targets: toolConfirm.targets } : null)
        setPendingRelease(
          releaseConfirm
            ? { id: releaseConfirm.id, tools: releaseConfirm.tools, targets: releaseConfirm.targets }
            : null,
        )
      } catch {
        // 다음 폴링에서 다시 시도
      }
    }, 1000)

    return () => window.clearInterval(poll)
  }, [])

  const respondTo = useCallback(async (id: number, confirmed: boolean) => {
    await fetch(`${API_BASE}/api/voice/requests/${id}/respond`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ confirmed }),
    })
  }, [])

  const respond = useCallback(
    (confirmed: boolean) => {
      if (!pending) return
      setPending(null)
      void respondTo(pending.id, confirmed)
    },
    [pending, respondTo],
  )

  const respondRelease = useCallback(
    (confirmed: boolean) => {
      if (!pendingRelease) return
      setPendingRelease(null)
      void respondTo(pendingRelease.id, confirmed)
    },
    [pendingRelease, respondTo],
  )

  return { connected, pending, pendingRelease, respond, respondRelease }
}
