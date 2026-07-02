import type { AuthUser } from '../types/auth'
import { request, setToken } from './client'

interface LoginResponse {
  access_token: string
  token_type: string
  user: AuthUser
}

export async function login(username: string, password: string): Promise<AuthUser> {
  const data = await request<LoginResponse>('/api/auth/login', {
    method: 'POST',
    body: JSON.stringify({ username, password }),
  })
  setToken(data.access_token)
  return data.user
}

export function me(): Promise<AuthUser> {
  return request<AuthUser>('/api/auth/me')
}

export function logout() {
  setToken(null)
}
