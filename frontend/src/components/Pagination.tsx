export function Pagination({
  page,
  pageSize,
  total,
  onPageChange,
}: {
  page: number
  pageSize: number
  total: number
  onPageChange: (page: number) => void
}) {
  if (total === 0) return null

  const totalPages = Math.max(1, Math.ceil(total / pageSize))
  const start = (page - 1) * pageSize + 1
  const end = Math.min(page * pageSize, total)

  return (
    <div className="pagination">
      <span className="pagination-info">
        {start}-{end} / 총 {total.toLocaleString()}건
      </span>
      <div className="pagination-controls">
        <button
          type="button"
          className="icon-btn"
          disabled={page <= 1}
          onClick={() => onPageChange(page - 1)}
          aria-label="이전 페이지"
        >
          ‹
        </button>
        <span className="pagination-page">
          {page} / {totalPages}
        </span>
        <button
          type="button"
          className="icon-btn"
          disabled={page >= totalPages}
          onClick={() => onPageChange(page + 1)}
          aria-label="다음 페이지"
        >
          ›
        </button>
      </div>
    </div>
  )
}
