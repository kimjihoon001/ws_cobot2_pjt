const TOKEN_KEY = 'cobot_token'

export function getToken(): string | null {
  return localStorage.getItem(TOKEN_KEY)
}

export function setToken(token: string | null) {
  if (token) localStorage.setItem(TOKEN_KEY, token)
  else localStorage.removeItem(TOKEN_KEY)
}

export async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const token = getToken()
  const headers: Record<string, string> = {
    'Content-Type': 'application/json',
    ...(init?.headers as Record<string, string> | undefined),
  }
  if (token) headers.Authorization = `Bearer ${token}`

  const res = await fetch(path, { ...init, headers })

  if (!res.ok) {
    const body = await res.json().catch(() => null)
    if (res.status === 401 && token) {
      setToken(null)
      window.dispatchEvent(new Event('auth:unauthorized'))
    }
    throw new Error(body?.detail ?? `요청 실패 (${res.status})`)
  }
  if (res.status === 204) return undefined as T
  return res.json() as Promise<T>
}
