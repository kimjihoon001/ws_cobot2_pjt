import type { InspectionMapData } from '../types/qc'

interface JengaMapProps {
  mapData: InspectionMapData | string | null
}

export function JengaMapVisualizer({ mapData }: JengaMapProps) {
  if (!mapData) {
    return <div className="qc-image-placeholder">선택된 검사 기록에 맵 데이터가 없습니다.</div>
  }

  let parsedMapData: InspectionMapData = {}
  try {
    parsedMapData = typeof mapData === 'string' ? JSON.parse(mapData) : mapData
  } catch (e) {
    return <div className="qc-image-placeholder">맵 데이터를 파싱할 수 없습니다.</div>
  }

  // Floors 6 to 1
  const floors = ["6", "5", "4", "3", "2", "1"]

  // 렌더링 헬퍼 함수
  const renderSmallBlock = (status: string, key: number) => {
    const isMissing = status === "X";
    return (
      <div key={key} style={{
        width: '40px', height: '32px',
        background: isMissing ? 'transparent' : 'linear-gradient(135deg, #deb887 0%, #faedcd 100%)',
        border: isMissing ? '2px dashed var(--status-critical)' : '1px solid #bc8f8f',
        borderRadius: '4px',
        boxShadow: isMissing ? 'none' : 'inset -2px -2px 4px rgba(0,0,0,0.1), 1px 2px 3px rgba(0,0,0,0.2)'
      }} />
    )
  }

  const renderLongBlock = (status: string, key: number) => {
    const isMissing = status === "X";
    return (
      <div key={key} style={{
        width: '132px', height: '32px',
        background: isMissing ? 'transparent' : 'linear-gradient(135deg, #deb887 0%, #faedcd 100%)',
        border: isMissing ? '2px dashed var(--status-critical)' : '1px solid #bc8f8f',
        borderRadius: '4px',
        boxShadow: isMissing ? 'none' : 'inset -2px -2px 4px rgba(0,0,0,0.1), 1px 2px 3px rgba(0,0,0,0.2)'
      }} />
    )
  }

  const renderFace = (title: string, faceIndex: number) => (
    <div key={faceIndex} style={{ padding: '15px', background: 'var(--surface-color)', borderRadius: '12px', border: '1px solid var(--border-color)', display: 'flex', flexDirection: 'column', gap: '8px', alignItems: 'center' }}>
      <h4 style={{ margin: '0 0 10px 0', color: 'var(--text-color)', fontSize: '1rem' }}>{title}</h4>
      {floors.map((floor) => {
        const floorNum = parseInt(floor)
        const blocks = parsedMapData[floor] || ["O", "O", "O"]
        const isOdd = floorNum % 2 !== 0
        
        let viewMode: 'short' | 'long'
        let blockArray: string[] = []
        let longStatus: string = "O"

        // 면(Face)의 방향에 따라 보이는 블록(외곽)을 결정
        if (faceIndex === 1) { // Face 1 (Front)
          if (isOdd) { viewMode = 'short'; blockArray = [blocks[0], blocks[1], blocks[2]] }
          else       { viewMode = 'long'; longStatus = blocks[0] }
        } else if (faceIndex === 2) { // Face 2 (Right)
          if (isOdd) { viewMode = 'long'; longStatus = blocks[2] }
          else       { viewMode = 'short'; blockArray = [blocks[0], blocks[1], blocks[2]] }
        } else if (faceIndex === 3) { // Face 3 (Back)
          if (isOdd) { viewMode = 'short'; blockArray = [blocks[2], blocks[1], blocks[0]] }
          else       { viewMode = 'long'; longStatus = blocks[2] }
        } else { // Face 4 (Left)
          if (isOdd) { viewMode = 'long'; longStatus = blocks[0] }
          else       { viewMode = 'short'; blockArray = [blocks[2], blocks[1], blocks[0]] }
        }

        return (
          <div key={floor} style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
            <div style={{ width: '24px', fontWeight: 'bold', color: 'var(--text-muted)', fontSize: '0.9rem' }}>{floor}F</div>
            {viewMode === 'short' 
              ? blockArray.map((b, idx) => renderSmallBlock(b, idx))
              : renderLongBlock(longStatus, 0)
            }
          </div>
        )
      })}
    </div>
  )

  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '10px', marginTop: '20px' }}>
      <h3 style={{ margin: 0, color: 'var(--text-color)' }}>젠가 검사 맵 시각화 (YOLO)</h3>
      <div style={{ display: 'flex', gap: '20px', width: '100%', justifyContent: 'center', flexWrap: 'wrap' }}>
        {renderFace('면 1 (Front)', 1)}
        {renderFace('면 2 (Right)', 2)}
        {renderFace('면 3 (Back)', 3)}
        {renderFace('면 4 (Left)', 4)}
      </div>
      <p className="empty-state" style={{ marginTop: '0', marginBottom: '10px' }}>선택된 항목의 6층 젠가 블록 상태입니다.</p>

      {parsedMapData.images && Array.isArray(parsedMapData.images) && parsedMapData.images.length > 0 && (
        <div style={{ width: '100%', marginTop: '10px', display: 'flex', flexDirection: 'column', gap: '10px' }}>
          <h4 style={{ margin: 0, color: 'var(--text-color)', alignSelf: 'flex-start' }}>측면 촬영 이미지</h4>
          <div style={{ display: 'flex', gap: '15px', overflowX: 'auto', paddingBottom: '15px' }}>
            {parsedMapData.images.map((imgUrl, idx) => (
              <img 
                key={idx} 
                src={imgUrl?.startsWith('/') ? `http://127.0.0.1:8000${imgUrl}` : imgUrl}
                alt={`Inspection view ${idx}`}
                style={{ height: '400px', borderRadius: '8px', border: '1px solid var(--border-color)', objectFit: 'cover', flexShrink: 0 }}
              />
            ))}
          </div>
        </div>
      )}
    </div>
  )
}
