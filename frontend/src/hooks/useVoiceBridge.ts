import { useCallback, useEffect, useRef, useState } from 'react'
import * as robotApi from '../api/robot'

export interface PendingConfirm {
  id: number
  tools: string[]
  targets: string[]
}

export interface HmiAlertAction {
  label: string
  command: 'retry_jenga' | 'retry_tool' | 'cancel_task' | 'dismiss'
  variant?: 'primary' | 'danger' | 'secondary'
}

export interface HmiAlert {
  id: number
  kind: string
  title: string
  message: string
  imageUrl?: string
  actions: HmiAlertAction[]
}

const API_BASE = 'http://127.0.0.1:8000'

function toHmiAlert(payload: robotApi.HmiAlertPayload): HmiAlert {
  return {
    id: payload.id ?? Date.now(),
    kind: payload.kind ?? 'generic',
    title: payload.title ?? '경고',
    message: payload.message ?? String(payload.message ?? ''),
    imageUrl: payload.image_url,
    actions: payload.actions ?? [{ label: '확인', command: 'dismiss', variant: 'primary' }],
  }
}

export function useVoiceBridge(url = 'ws://127.0.0.1:8000/ws') {
  const wsRef = useRef<WebSocket | null>(null)
  const [pending, setPending] = useState<PendingConfirm | null>(null)
  const [pendingRelease, setPendingRelease] = useState<PendingConfirm | null>(null)
  const [connected, setConnected] = useState(false)
  const [hmiAlert, setHmiAlert] = useState<HmiAlert | null>(null)
  const lastAlertIdRef = useRef<number | null>(null)

  // WS는 폴링 주기(1초)보다 빠르게 새 요청이 생겼다는 걸 알아채는 용도로만 사용 —
  // 실제 정답 소스와 응답 제출은 전부 DB 기반 REST(/api/voice/requests/*)로 처리한다.
  useEffect(() => {
    let stopped = false
    let retryTimer: number | undefined

    const connect = () => {
      const ws = new WebSocket(url)
      wsRef.current = ws

      ws.onopen = () => setConnected(true)
      ws.onmessage = (event) => {
        try {
          const payload = JSON.parse(event.data)
          if (payload.type === 'alert') {
            const alert = toHmiAlert(payload)
            lastAlertIdRef.current = alert.id
            setHmiAlert(alert)
          }
        } catch {}
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
        const data = await robotApi.getLatestAlert()
        if (!data.alert) return
        const alert = toHmiAlert(data.alert)
        if (lastAlertIdRef.current === alert.id) return
        lastAlertIdRef.current = alert.id
        setHmiAlert(alert)
      } catch {
        // WS가 살아 있으면 다음 메시지로 보완되고, 아니면 다음 폴링에서 재시도한다.
      }
    }, 1000)

    return () => window.clearInterval(poll)
  }, [])

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

  return { connected, pending, pendingRelease, respond, respondRelease, hmiAlert }
}
