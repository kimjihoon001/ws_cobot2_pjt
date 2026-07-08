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

  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: '10px', marginTop: '20px' }}>
      <h3 style={{ margin: 0, color: 'var(--text-color)' }}>젠가 검사 맵 시각화 (YOLO)</h3>
      <div style={{ display: 'flex', gap: '30px', width: '100%', justifyContent: 'center' }}>
        {/* Face 1 View */}
        <div style={{ padding: '20px', background: 'var(--surface-color)', borderRadius: '12px', border: '1px solid var(--border-color)', display: 'flex', flexDirection: 'column', gap: '8px', alignItems: 'center' }}>
          <h4 style={{ margin: '0 0 10px 0', color: 'var(--text-color)' }}>면 1 (Face 1)</h4>
          {floors.map((floor) => {
            const floorNum = parseInt(floor)
            const blocks = parsedMapData[floor] || ["O", "O", "O"]
            const isOdd = floorNum % 2 !== 0
            
            if (isOdd) {
              return (
                <div key={floor} style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
                  <div style={{ width: '24px', fontWeight: 'bold', color: 'var(--text-muted)', fontSize: '0.9rem' }}>{floor}F</div>
                  {blocks.map((block, idx) => (
                    <div key={idx} style={{
                      width: '40px', height: '32px',
                      background: block === "X" ? 'transparent' : 'linear-gradient(135deg, #deb887 0%, #faedcd 100%)',
                      border: block === "X" ? '2px dashed var(--status-critical)' : '1px solid #bc8f8f',
                      borderRadius: '4px',
                      boxShadow: block === "X" ? 'none' : 'inset -2px -2px 4px rgba(0,0,0,0.1), 1px 2px 3px rgba(0,0,0,0.2)'
                    }} />
                  ))}
                </div>
              )
            } else {
              const allMissing = blocks.every((b: string) => b === "X")
              return (
                <div key={floor} style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
                  <div style={{ width: '24px', fontWeight: 'bold', color: 'var(--text-muted)', fontSize: '0.9rem' }}>{floor}F</div>
                  <div style={{
                    width: '132px', height: '32px',
                    background: allMissing ? 'transparent' : 'linear-gradient(135deg, #deb887 0%, #faedcd 100%)',
                    border: allMissing ? '2px dashed var(--status-critical)' : '1px solid #bc8f8f',
                    borderRadius: '4px',
                    boxShadow: allMissing ? 'none' : 'inset -2px -2px 4px rgba(0,0,0,0.1), 1px 2px 3px rgba(0,0,0,0.2)'
                  }} />
                </div>
              )
            }
          })}
        </div>

        {/* Face 2 View */}
        <div style={{ padding: '20px', background: 'var(--surface-color)', borderRadius: '12px', border: '1px solid var(--border-color)', display: 'flex', flexDirection: 'column', gap: '8px', alignItems: 'center' }}>
          <h4 style={{ margin: '0 0 10px 0', color: 'var(--text-color)' }}>면 2 (Face 2)</h4>
          {floors.map((floor) => {
            const floorNum = parseInt(floor)
            const blocks = parsedMapData[floor] || ["O", "O", "O"]
            const isOdd = floorNum % 2 !== 0
            
            if (!isOdd) {
              return (
                <div key={floor} style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
                  <div style={{ width: '24px', fontWeight: 'bold', color: 'var(--text-muted)', fontSize: '0.9rem' }}>{floor}F</div>
                  {blocks.map((block, idx) => (
                    <div key={idx} style={{
                      width: '40px', height: '32px',
                      background: block === "X" ? 'transparent' : 'linear-gradient(135deg, #deb887 0%, #faedcd 100%)',
                      border: block === "X" ? '2px dashed var(--status-critical)' : '1px solid #bc8f8f',
                      borderRadius: '4px',
                      boxShadow: block === "X" ? 'none' : 'inset -2px -2px 4px rgba(0,0,0,0.1), 1px 2px 3px rgba(0,0,0,0.2)'
                    }} />
                  ))}
                </div>
              )
            } else {
              const allMissing = blocks.every((b: string) => b === "X")
              return (
                <div key={floor} style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
                  <div style={{ width: '24px', fontWeight: 'bold', color: 'var(--text-muted)', fontSize: '0.9rem' }}>{floor}F</div>
                  <div style={{
                    width: '132px', height: '32px',
                    background: allMissing ? 'transparent' : 'linear-gradient(135deg, #deb887 0%, #faedcd 100%)',
                    border: allMissing ? '2px dashed var(--status-critical)' : '1px solid #bc8f8f',
                    borderRadius: '4px',
                    boxShadow: allMissing ? 'none' : 'inset -2px -2px 4px rgba(0,0,0,0.1), 1px 2px 3px rgba(0,0,0,0.2)'
                  }} />
                </div>
              )
            }
          })}
        </div>
      </div>
      <p className="empty-state" style={{ marginTop: '0', marginBottom: '10px' }}>선택된 항목의 6층 젠가 블록 상태입니다.</p>

      {parsedMapData.images && Array.isArray(parsedMapData.images) && parsedMapData.images.length > 0 && (
        <div style={{ width: '100%', marginTop: '10px', display: 'flex', flexDirection: 'column', gap: '10px' }}>
          <h4 style={{ margin: 0, color: 'var(--text-color)', alignSelf: 'flex-start' }}>측면 촬영 이미지</h4>
          <div style={{ display: 'flex', gap: '15px', overflowX: 'auto', paddingBottom: '15px' }}>
            {parsedMapData.images.map((imgUrl, idx) => (
              <img 
                key={idx} 
                src={imgUrl}
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
