import { useState } from 'react'
import { useAuth } from './auth/AuthContext'
import { LoginPage } from './components/LoginPage'
import { UsersPanel } from './components/UsersPanel'
import { InventoryPage } from './pages/InventoryPage'
import { RobotDashboardPage } from './pages/RobotDashboardPage'
import { QcPage } from './pages/QcPage'
import './App.css'

type Tab = 'inventory' | 'robot' | 'qc' | 'users'

function App() {
  const { user, loading, logout } = useAuth()
  const [tab, setTab] = useState<Tab>('inventory')

  if (loading) {
    return <p className="empty-state">불러오는 중...</p>
  }

  if (!user) {
    return <LoginPage />
  }

  const canManage = user.role === 'admin'

  return (
    <div className="page">
      <header className="page-header">
        <div className="header-top">
          <div>
            <h1>자원 재고 관리</h1>
            <p className="page-subtitle">cobot2 협업 로봇 도구·자재 재고 현황</p>
          </div>
          <div className="header-user">
            <span>
              {user.username} ({canManage ? '관리자' : '작업자'})
            </span>
            <button type="button" className="text-btn" onClick={logout}>
              로그아웃
            </button>
          </div>
        </div>

        <nav className="tabs">
          <button
            type="button"
            className={tab === 'inventory' ? 'tab tab-active' : 'tab'}
            onClick={() => setTab('inventory')}
          >
            재고 관리
          </button>
          <button
            type="button"
            className={tab === 'robot' ? 'tab tab-active' : 'tab'}
            onClick={() => setTab('robot')}
          >
            로봇 관리
          </button>
          <button
            type="button"
            className={tab === 'qc' ? 'tab tab-active' : 'tab'}
            onClick={() => setTab('qc')}
          >
            품질검사
          </button>
          {canManage && (
            <button
              type="button"
              className={tab === 'users' ? 'tab tab-active' : 'tab'}
              onClick={() => setTab('users')}
            >
              계정 관리
            </button>
          )}
        </nav>
      </header>

      {tab === 'users' && canManage ? (
        <UsersPanel />
      ) : tab === 'robot' ? (
        <RobotDashboardPage />
      ) : tab === 'qc' ? (
        <QcPage />
      ) : (
        <InventoryPage canManage={canManage} />
      )}
    </div>
  )
}

export default App
