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
