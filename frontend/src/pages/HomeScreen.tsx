import { useEffect, useState } from 'react'
import * as resourcesApi from '../api/resources'
import * as qcApi from '../api/qc'
import type { Summary } from '../types/resource'
import type { QcSummary } from '../types/qc'
import { VoiceStartButton } from '../components/VoiceStartButton'
import { DirectRobotActionButtons } from '../components/DirectRobotActionButtons'

type Screen = 'inventory' | 'qc' | 'robot' | 'work' | 'users'

export function HomeScreen({
  canManage,
  onNavigate,
}: {
  canManage: boolean
  onNavigate: (screen: Screen) => void
}) {
  const [invSummary, setInvSummary] = useState<Summary | null>(null)
  const [qcSummary, setQcSummary] = useState<QcSummary | null>(null)

  useEffect(() => {
    const refresh = () => {
      resourcesApi.getSummary().then(setInvSummary).catch(() => setInvSummary(null))
      qcApi.getQcSummary().then(setQcSummary).catch(() => setQcSummary(null))
    }
    refresh()
    // 홈에 머무는 동안에도 백엔드 재고/QC 변동이 반영되도록 주기적 갱신 + 창 포커스 시 갱신
    const timer = setInterval(refresh, 5000)
    window.addEventListener('focus', refresh)
    return () => {
      clearInterval(timer)
      window.removeEventListener('focus', refresh)
    }
  }, [])

  return (
    <div>
      <div className="launcher-head">
        <h1>무엇을 하시겠어요?</h1>
        <p>모듈을 선택하면 화면이 전환됩니다.</p>
        <div className="home-action-row">
          <VoiceStartButton />
          <DirectRobotActionButtons />
        </div>
      </div>

      <div className="tile-grid">
        <button type="button" className="tile" onClick={() => onNavigate('inventory')}>
          <div className="tile-icon" style={{ background: 'var(--accent)' }}>
            재고
          </div>
          <h3>재고 관리</h3>
          <p className="tile-desc">도구·자재 재고 현황</p>
          <div className="tile-stat">{invSummary ? invSummary.total.toLocaleString() : '-'}</div>
          <div className="tile-note">
            {invSummary ? `부족 ${invSummary.low} · 품절 ${invSummary.out}` : '불러오는 중...'}
          </div>
        </button>

        <button type="button" className="tile" onClick={() => onNavigate('qc')}>
          <div className="tile-icon" style={{ background: 'var(--status-good)' }}>
            QC
          </div>
          <h3>품질검사</h3>
          <p className="tile-desc">YOLO 기반 불량 위치 검사</p>
          <div className="tile-stat">{qcSummary ? qcSummary.total.toLocaleString() : '-'}</div>
          <div className="tile-note">
            {qcSummary ? `PASS ${qcSummary.passed} · FAIL ${qcSummary.failed}` : '불러오는 중...'}
          </div>
        </button>

        <button type="button" className="tile" onClick={() => onNavigate('robot')}>
          <div className="tile-icon" style={{ background: 'var(--status-critical)' }}>
            로봇
          </div>
          <h3>로봇 관리</h3>
          <p className="tile-desc">연결 · 제어 · 안전관리</p>
          <div className="tile-stat">미연결</div>
          <div className="tile-note">현재 작업: 대기</div>
        </button>

        <button type="button" className="tile" onClick={() => onNavigate('work')}>
          <div className="tile-icon" style={{ background: 'var(--status-warning)' }}>
            작업
          </div>
          <h3>작업 화면</h3>
          <p className="tile-desc">현재 작업 현황 및 제어</p>
          <div className="tile-stat">대기 중</div>
        </button>

        {canManage && (
          <button type="button" className="tile" onClick={() => onNavigate('users')}>
            <div className="tile-icon" style={{ background: 'var(--text-secondary)' }}>
              계정
            </div>
            <h3>계정 관리</h3>
            <p className="tile-desc">사용자 계정 및 권한</p>
          </button>
        )}
      </div>
    </div>
  )
}
