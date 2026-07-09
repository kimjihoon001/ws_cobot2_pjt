import { useEffect, useState } from 'react'
import * as robotApi from '../api/robot'
import type { RobotStatus } from '../api/robot'

const FALLBACK_STATUS: RobotStatus = {
  connected: false,
  mode: 'unknown',
  controller: '미연결',
  current_task: '대기',
  task_key: 'idle',
  checks: {
    dsr: false,
    moveit: false,
    jenga_inspector: false,
    tool_pick: false,
    voice: false,
    hand: false,
  },
  estop: false,
  estop_message: '',
  joints: {
    joint_1: null,
    joint_2: null,
    joint_3: null,
    joint_4: null,
    joint_5: null,
    joint_6: null,
    gripper: null,
  },
  joint_units: {
    joint_1: 'deg',
    joint_2: 'deg',
    joint_3: 'deg',
    joint_4: 'deg',
    joint_5: 'deg',
    joint_6: 'deg',
    gripper: 'mm',
  },
  last_pick_task: '',
  ros_bridge: false,
}

export function useRobotStatus(intervalMs = 2000) {
  const [status, setStatus] = useState<RobotStatus>(FALLBACK_STATUS)

  useEffect(() => {
    let cancelled = false

    const refresh = () => {
      robotApi
        .getStatus()
        .then((next) => {
          if (!cancelled) setStatus(next)
        })
        .catch(() => {
          if (!cancelled) setStatus(FALLBACK_STATUS)
        })
    }

    refresh()
    const timer = window.setInterval(refresh, intervalMs)
    window.addEventListener('focus', refresh)
    return () => {
      cancelled = true
      window.clearInterval(timer)
      window.removeEventListener('focus', refresh)
    }
  }, [intervalMs])

  return status
}
