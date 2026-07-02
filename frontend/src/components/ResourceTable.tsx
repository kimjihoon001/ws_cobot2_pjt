import type { Resource } from '../types/resource'
import { StatusBadge } from './StatusBadge'

interface Props {
  resources: Resource[]
  canManage: boolean
  onAdjust: (resource: Resource, delta: number) => void
  onEdit: (resource: Resource) => void
  onDelete: (resource: Resource) => void
}

export function ResourceTable({ resources, canManage, onAdjust, onEdit, onDelete }: Props) {
  if (resources.length === 0) {
    return <p className="empty-state">등록된 자원이 없습니다. 상단에서 자원을 추가하세요.</p>
  }

  return (
    <table className="resource-table">
      <thead>
        <tr>
          <th>번호</th>
          <th>이름</th>
          <th>분류</th>
          <th>위치</th>
          <th className="num-col">수량</th>
          <th className="num-col">최소수량</th>
          <th>상태</th>
          <th>관리</th>
        </tr>
      </thead>
      <tbody>
        {resources.map((r) => (
          <tr key={r.id}>
            <td className="num-col">{r.id}</td>
            <td>{r.name}</td>
            <td>{r.category || '-'}</td>
            <td>{r.location || '-'}</td>
            <td className="num-col">
              {r.quantity.toLocaleString()} {r.unit}
            </td>
            <td className="num-col">{r.min_quantity.toLocaleString()}</td>
            <td>
              <StatusBadge status={r.status} />
            </td>
            <td>
              <div className="row-actions">
                <button
                  type="button"
                  className="icon-btn"
                  onClick={() => onAdjust(r, -1)}
                  disabled={r.quantity <= 0}
                  aria-label="수량 1 감소"
                >
                  −
                </button>
                <button
                  type="button"
                  className="icon-btn"
                  onClick={() => onAdjust(r, 1)}
                  aria-label="수량 1 증가"
                >
                  +
                </button>
                {canManage && (
                  <>
                    <button type="button" className="text-btn" onClick={() => onEdit(r)}>
                      수정
                    </button>
                    <button
                      type="button"
                      className="text-btn danger"
                      onClick={() => onDelete(r)}
                    >
                      삭제
                    </button>
                  </>
                )}
              </div>
            </td>
          </tr>
        ))}
      </tbody>
    </table>
  )
}
