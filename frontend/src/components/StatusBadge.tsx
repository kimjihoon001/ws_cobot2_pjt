import type { ResourceStatus } from '../types/resource'

const STATUS_META: Record<ResourceStatus, { label: string; color: string }> = {
  normal: { label: '정상', color: 'var(--status-good)' },
  low: { label: '부족', color: 'var(--status-warning)' },
  out: { label: '품절', color: 'var(--status-critical)' },
}

export function StatusBadge({ status }: { status: ResourceStatus }) {
  const meta = STATUS_META[status]
  return (
    <span
      className="status-badge"
      style={{
        background: `color-mix(in srgb, ${meta.color} 14%, transparent)`,
        borderColor: `color-mix(in srgb, ${meta.color} 40%, transparent)`,
      }}
    >
      <span className="status-dot" style={{ background: meta.color }} />
      {meta.label}
    </span>
  )
}
