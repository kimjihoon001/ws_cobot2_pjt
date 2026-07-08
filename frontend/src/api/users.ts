import type { AuthUser, UserRole } from '../types/auth'
import { request } from './client'

export function listUsers(): Promise<AuthUser[]> {
  return request<AuthUser[]>('/api/users')
}

export function createUser(data: {
  username: string
  password: string
  role: UserRole
}): Promise<AuthUser> {
  return request<AuthUser>('/api/users', { method: 'POST', body: JSON.stringify(data) })
}

export function deleteUser(id: number): Promise<void> {
  return request<void>(`/api/users/${id}`, { method: 'DELETE' })
}
