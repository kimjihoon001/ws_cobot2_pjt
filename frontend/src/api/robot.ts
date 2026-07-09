import { request } from './client'

export interface RobotChecks {
  dsr: boolean
  moveit: boolean
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
  last_pick_task: string
  ros_bridge: boolean
}

export function getStatus(): Promise<RobotStatus> {
  return request<RobotStatus>('/api/robot/status')
}
