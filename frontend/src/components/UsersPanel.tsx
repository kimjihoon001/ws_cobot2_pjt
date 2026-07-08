import { useEffect, useState } from 'react'
import type { FormEvent } from 'react'
import type { AuthUser, UserRole } from '../types/auth'
import * as usersApi from '../api/users'
import { useAuth } from '../auth/AuthContext'

export function UsersPanel() {
  const { user: currentUser } = useAuth()
  const [users, setUsers] = useState<AuthUser[]>([])
  const [username, setUsername] = useState('')
  const [password, setPassword] = useState('')
  const [role, setRole] = useState<UserRole>('worker')
  const [error, setError] = useState<string | null>(null)
  const [loading, setLoading] = useState(true)

  async function refresh() {
    try {
      setUsers(await usersApi.listUsers())
      setError(null)
    } catch (err) {
      setError(err instanceof Error ? err.message : '계정 목록을 불러오지 못했습니다.')
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    refresh()
  }, [])

  async function handleCreate(e: FormEvent) {
    e.preventDefault()
    setError(null)
    try {
      await usersApi.createUser({ username, password, role })
      setUsername('')
      setPassword('')
      setRole('worker')
      await refresh()
    } catch (err) {
      setError(err instanceof Error ? err.message : '계정 생성에 실패했습니다.')
    }
  }

  async function handleDelete(id: number) {
    if (!confirm('이 계정을 삭제하시겠습니까?')) return
    try {
      await usersApi.deleteUser(id)
      await refresh()
    } catch (err) {
      setError(err instanceof Error ? err.message : '삭제에 실패했습니다.')
    }
  }

  return (
    <div>
      <form className="user-form" onSubmit={handleCreate}>
        <input
          placeholder="아이디 (3자 이상)"
          value={username}
          onChange={(e) => setUsername(e.target.value)}
          minLength={3}
          required
        />
        <input
          placeholder="비밀번호 (6자 이상)"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          minLength={6}
          required
        />
        <select value={role} onChange={(e) => setRole(e.target.value as UserRole)}>
          <option value="worker">작업자</option>
          <option value="admin">관리자</option>
        </select>
        <button type="submit" className="primary-btn">
          계정 추가
        </button>
      </form>

      {error && <p className="form-error">{error}</p>}

      {loading ? (
        <p className="empty-state">불러오는 중...</p>
      ) : (
        <table className="resource-table">
          <thead>
            <tr>
              <th>번호</th>
              <th>아이디</th>
              <th>역할</th>
              <th>관리</th>
            </tr>
          </thead>
          <tbody>
            {users.map((u) => (
              <tr key={u.id}>
                <td className="num-col">{u.id}</td>
                <td>{u.username}</td>
                <td>{u.role === 'admin' ? '관리자' : '작업자'}</td>
                <td>
                  <button
                    type="button"
                    className="text-btn danger"
                    onClick={() => handleDelete(u.id)}
                    disabled={u.id === currentUser?.id}
                  >
                    삭제
                  </button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </div>
  )
}
