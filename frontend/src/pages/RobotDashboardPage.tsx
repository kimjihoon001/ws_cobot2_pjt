const JOINTS = [
  { name: 'joint_1', angle: '-' },
  { name: 'joint_2', angle: '-' },
  { name: 'joint_3', angle: '-' },
  { name: 'joint_4', angle: '-' },
  { name: 'joint_5', angle: '-' },
  { name: 'joint_6', angle: '-' },
  { name: 'gripper', angle: '-' },
]

export function RobotDashboardPage() {
  return (
    <div>
      <div className="summary-cards">
        <div className="stat-tile">
          <div className="stat-tile-label">
            <span className="status-dot" style={{ background: 'var(--status-critical)' }} />
            연결 상태
          </div>
          <div className="stat-tile-value">미연결</div>
        </div>
        <div className="stat-tile">
          <div className="stat-tile-label">모드</div>
          <div className="stat-tile-value">-</div>
        </div>
        <div className="stat-tile">
          <div className="stat-tile-label">현재 작업</div>
          <div className="stat-tile-value">대기</div>
        </div>
        <div className="stat-tile">
          <div className="stat-tile-label">컨트롤러</div>
          <div className="stat-tile-value">-</div>
        </div>
      </div>

      <table className="resource-table">
        <thead>
          <tr>
            <th>조인트</th>
            <th className="num-col">각도</th>
          </tr>
        </thead>
        <tbody>
          {JOINTS.map((joint) => (
            <tr key={joint.name}>
              <td>{joint.name}</td>
              <td className="num-col">{joint.angle}</td>
            </tr>
          ))}
        </tbody>
      </table>

      <p className="empty-state">ROS2/MoveIt 연동 후 실시간 로봇 상태가 여기에 표시됩니다.</p>
    </div>
  )
}
