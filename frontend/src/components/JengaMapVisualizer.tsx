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
      <div style={{ padding: '20px', background: 'var(--surface-color)', borderRadius: '12px', border: '1px solid var(--border-color)', display: 'flex', flexDirection: 'column', gap: '8px' }}>
        {floors.map((floor) => {
          const blocks = parsedMapData[floor] || ["O", "O", "O"]
          return (
            <div key={floor} style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
              <div style={{ width: '40px', display: 'flex', alignItems: 'center', fontWeight: 'bold', color: 'var(--text-muted)' }}>
                {floor}층
              </div>
              {blocks.map((block, idx) => {
                const isMissing = block === "X"
                return (
                  <div
                    key={idx}
                    style={{
                      width: '70px',
                      height: '35px',
                      background: isMissing ? 'transparent' : 'var(--primary-color)',
                      border: isMissing ? '2px dashed var(--status-critical)' : '2px solid color-mix(in srgb, var(--primary-color) 40%, black)',
                      borderRadius: '6px',
                      display: 'flex',
                      alignItems: 'center',
                      justifyContent: 'center',
                      color: isMissing ? 'var(--status-critical)' : '#fff',
                      fontWeight: 'bold',
                      opacity: isMissing ? 0.8 : 1,
                      boxShadow: isMissing ? 'none' : '0 2px 4px rgba(0,0,0,0.2)'
                    }}
                  >
                    {isMissing ? '누락' : '정상'}
                  </div>
                )
              })}
            </div>
          )
        })}
      </div>
      <p className="empty-state" style={{ marginTop: '0', marginBottom: '10px' }}>선택된 항목의 6층 젠가 블록 상태입니다.</p>

      {parsedMapData.images && Array.isArray(parsedMapData.images) && parsedMapData.images.length > 0 && (
        <div style={{ width: '100%', marginTop: '10px', display: 'flex', flexDirection: 'column', gap: '10px' }}>
          <h4 style={{ margin: 0, color: 'var(--text-color)', alignSelf: 'flex-start' }}>측면 촬영 이미지</h4>
          <div style={{ display: 'flex', gap: '10px', overflowX: 'auto', paddingBottom: '10px' }}>
            {parsedMapData.images.map((imgUrl, idx) => (
              <img 
                key={idx} 
                src={`http://localhost:8000${imgUrl}`} 
                alt={`Inspection view ${idx}`}
                style={{ height: '200px', borderRadius: '8px', border: '1px solid var(--border-color)', objectFit: 'cover' }}
              />
            ))}
          </div>
        </div>
      )}
    </div>
  )
}
