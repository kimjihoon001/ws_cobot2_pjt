import type { SortDir } from '../utils/sort'

export function SortableTh<K extends string>({
  label,
  sortKey,
  activeKey,
  dir,
  onSort,
  className,
}: {
  label: string
  sortKey: K
  activeKey: K
  dir: SortDir
  onSort: (key: K) => void
  className?: string
}) {
  const active = sortKey === activeKey
  return (
    <th className={className}>
      <button type="button" className="sort-th-btn" onClick={() => onSort(sortKey)}>
        {label}
        <span className={active ? 'sort-arrow sort-arrow-active' : 'sort-arrow'}>
          {active && dir === 'desc' ? '▼' : '▲'}
        </span>
      </button>
    </th>
  )
}
