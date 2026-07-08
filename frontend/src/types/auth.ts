export type UserRole = 'admin' | 'worker'

export interface AuthUser {
  id: number
  username: string
  role: UserRole
  created_at: string
}
