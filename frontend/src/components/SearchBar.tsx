import type { ResourceStatus } from '../types/resource'

interface Props {
  search: string
  onSearchChange: (value: string) => void
  status: ResourceStatus | ''
  onStatusChange: (value: ResourceStatus | '') => void
  category: string
  onCategoryChange: (value: string) => void
  categories: string[]
  canAdd: boolean
  onAddClick: () => void
}

export function SearchBar({
  search,
  onSearchChange,
  status,
  onStatusChange,
  category,
  onCategoryChange,
  categories,
  canAdd,
  onAddClick,
}: Props) {
  return (
    <div className="search-bar">
      <input
        className="search-input"
        placeholder="이름 또는 위치로 검색"
        value={search}
        onChange={(e) => onSearchChange(e.target.value)}
      />
      <select value={category} onChange={(e) => onCategoryChange(e.target.value)}>
        <option value="">전체 분류</option>
        {categories.map((c) => (
          <option key={c} value={c}>
            {c}
          </option>
        ))}
      </select>
      <select
        value={status}
        onChange={(e) => onStatusChange(e.target.value as ResourceStatus | '')}
      >
        <option value="">전체 상태</option>
        <option value="normal">정상</option>
        <option value="low">부족</option>
        <option value="out">품절</option>
      </select>
      {canAdd && (
        <button type="button" className="primary-btn" onClick={onAddClick}>
          + 자원 추가
        </button>
      )}
    </div>
  )
}
