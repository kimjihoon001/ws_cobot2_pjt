import type { Resource } from '../types/resource'
import { StatusBadge } from './StatusBadge'
import { SortableTh } from './SortableTh'
import type { SortDir } from '../utils/sort'

export type ResourceSortKey =
  | 'id'
  | 'name'
  | 'item_type'
  | 'category'
  | 'location'
  | 'quantity'
  | 'min_quantity'
  | 'status'

const ITEM_TYPE_LABEL: Record<Resource['item_type'], string> = {
  material: '원자재',
  product: '완제품',
}

interface Props {
  resources: Resource[]
  canManage: boolean
  sortKey: ResourceSortKey
  sortDir: SortDir
  onSort: (key: ResourceSortKey) => void
  onAdjust: (resource: Resource, delta: number) => void
  onEdit: (resource: Resource) => void
  onDelete: (resource: Resource) => void
}

export function ResourceTable({
  resources,
  canManage,
  sortKey,
  sortDir,
  onSort,
  onAdjust,
  onEdit,
  onDelete,
}: Props) {
  if (resources.length === 0) {
    return <p className="empty-state">등록된 자원이 없습니다. 상단에서 자원을 추가하세요.</p>
  }

  return (
    <table className="resource-table">
      <thead>
        <tr>
          <SortableTh label="번호" sortKey="id" activeKey={sortKey} dir={sortDir} onSort={onSort} className="num-col" />
          <SortableTh label="이름" sortKey="name" activeKey={sortKey} dir={sortDir} onSort={onSort} />
          <SortableTh label="구분" sortKey="item_type" activeKey={sortKey} dir={sortDir} onSort={onSort} />
          <SortableTh label="분류" sortKey="category" activeKey={sortKey} dir={sortDir} onSort={onSort} />
          <SortableTh label="위치" sortKey="location" activeKey={sortKey} dir={sortDir} onSort={onSort} />
          <SortableTh label="수량" sortKey="quantity" activeKey={sortKey} dir={sortDir} onSort={onSort} className="num-col" />
          <SortableTh label="최소수량" sortKey="min_quantity" activeKey={sortKey} dir={sortDir} onSort={onSort} className="num-col" />
          <SortableTh label="상태" sortKey="status" activeKey={sortKey} dir={sortDir} onSort={onSort} />
          <th>관리</th>
        </tr>
      </thead>
      <tbody>
        {resources.map((r) => (
          <tr key={r.id}>
            <td className="num-col">{r.id}</td>
            <td>{r.name}</td>
            <td>{ITEM_TYPE_LABEL[r.item_type]}</td>
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
