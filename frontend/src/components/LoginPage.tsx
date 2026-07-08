import { useState } from 'react'
import type { FormEvent } from 'react'
import { useAuth } from '../auth/AuthContext'

export function LoginPage() {
  const { login } = useAuth()
  const [username, setUsername] = useState('')
  const [password, setPassword] = useState('')
  const [error, setError] = useState<string | null>(null)
  const [submitting, setSubmitting] = useState(false)

  async function handleSubmit(e: FormEvent) {
    e.preventDefault()
    setSubmitting(true)
    setError(null)
    try {
      await login(username, password)
    } catch (err) {
      setError(err instanceof Error ? err.message : '로그인에 실패했습니다.')
    } finally {
      setSubmitting(false)
    }
  }

  return (
    <div className="login-page">
      <form className="login-card" onSubmit={handleSubmit}>
        <h1>cobot2 HMI</h1>
        <p className="page-subtitle">로그인이 필요합니다</p>

        <label>
          아이디
          <input value={username} onChange={(e) => setUsername(e.target.value)} autoFocus />
        </label>

        <label>
          비밀번호
          <input
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
          />
        </label>

        {error && <p className="form-error">{error}</p>}

        <button type="submit" className="primary-btn" disabled={submitting}>
          {submitting ? '로그인 중...' : '로그인'}
        </button>
      </form>
    </div>
  )
}
