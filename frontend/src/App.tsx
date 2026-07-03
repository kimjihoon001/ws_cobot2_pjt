import { useState } from 'react'
import { useAuth } from './auth/AuthContext'
import { LoginPage } from './components/LoginPage'
import { UsersPanel } from './components/UsersPanel'
import { HomeScreen } from './pages/HomeScreen'
import { InventoryPage } from './pages/InventoryPage'
import { RobotDashboardPage } from './pages/RobotDashboardPage'
import { QcPage } from './pages/QcPage'
import './App.css'

type Screen = 'home' | 'inventory' | 'qc' | 'robot' | 'users'

const SCREEN_TITLE: Record<Screen, string> = {
  home: '홈',
  inventory: '재고 관리',
  qc: '품질검사',
  robot: '로봇 관리',
  users: '계정 관리',
}

function App() {
  const { user, loading, logout } = useAuth()
  const [screen, setScreen] = useState<Screen>('home')

  if (loading) {
    return <p className="empty-state">불러오는 중...</p>
  }

  if (!user) {
    return <LoginPage />
  }

  const canManage = user.role === 'admin'

  return (
    <div>
      <header className="app-topbar">
        <div className="brand">cobot2 HMI</div>
        <div className="user-chip">
          <span>
            {user.username} ({canManage ? '관리자' : '작업자'})
          </span>
          <button type="button" className="text-btn" onClick={logout}>
            로그아웃
          </button>
        </div>
      </header>

      {screen !== 'home' && (
        <div className="screen-topbar">
          <button type="button" className="back-btn" onClick={() => setScreen('home')}>
            ← 홈
          </button>
          <h1>{SCREEN_TITLE[screen]}</h1>
        </div>
      )}

      <div className="content">
        {screen === 'home' && <HomeScreen canManage={canManage} onNavigate={setScreen} />}
        {screen === 'inventory' && <InventoryPage canManage={canManage} />}
        {screen === 'qc' && <QcPage />}
        {screen === 'robot' && <RobotDashboardPage />}
        {screen === 'users' && canManage && <UsersPanel />}
      </div>
    </div>
  )
}

export default App
