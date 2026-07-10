import { request } from './client'

export interface RobotChecks {
  dsr: boolean
  moveit: boolean
  conveyor: boolean
  jenga_inspector: boolean
  tool_pick: boolean
  voice: boolean
  hand: boolean
}

export interface RobotStatus {
  connected: boolean
  mode: string
  controller: string
  current_task: string
  task_key: string
  checks: RobotChecks
  estop: boolean
  estop_message: string
  joints: Record<string, number | null>
  joint_units: Record<string, string>
  last_pick_task: string
  ros_bridge: boolean
}

export function getStatus(): Promise<RobotStatus> {
  return request<RobotStatus>('/api/robot/status')
}

export interface RobotCommandResult {
  success: boolean
  estop?: boolean
  message: string
}

export interface RobotStartResult {
  started: boolean
  message: string
}

export interface HmiAlertAction {
  label: string
  command: 'retry_jenga' | 'retry_tool' | 'cancel_task' | 'dismiss'
  variant?: 'primary' | 'danger' | 'secondary'
}

export interface HmiAlertPayload {
  id: number
  kind?: string
  title?: string
  message?: string
  image_url?: string
  actions?: HmiAlertAction[]
}

export function emergencyStop(): Promise<RobotCommandResult> {
  return request<RobotCommandResult>('/api/robot/emergency_stop', { method: 'POST' })
}

export function releaseEstop(): Promise<RobotCommandResult> {
  return request<RobotCommandResult>('/api/robot/release_estop', { method: 'POST' })
}

export function moveHome(): Promise<RobotCommandResult> {
  return request<RobotCommandResult>('/api/robot/move_home', { method: 'POST' })
}

export function openGripper(): Promise<RobotCommandResult> {
  return request<RobotCommandResult>('/api/robot/open_gripper', { method: 'POST' })
}

export function closeGripper(): Promise<RobotCommandResult> {
  return request<RobotCommandResult>('/api/robot/close_gripper', { method: 'POST' })
}

export function runInspection(): Promise<RobotStartResult> {
  return request<RobotStartResult>('/api/robot/run_inspection', { method: 'POST' })
}

export function retryPickTask(): Promise<RobotCommandResult> {
  return request<RobotCommandResult>('/api/robot/retry_pick_task', { method: 'POST' })
}

export function cancelTask(): Promise<RobotCommandResult> {
  return request<RobotCommandResult>('/api/robot/cancel_task', { method: 'POST' })
}

export function getLatestAlert(): Promise<{ alert: HmiAlertPayload | null }> {
  return request<{ alert: HmiAlertPayload | null }>('/api/robot/alerts/latest')
}
