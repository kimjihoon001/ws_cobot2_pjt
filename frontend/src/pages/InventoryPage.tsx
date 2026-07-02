import { useEffect, useState, useCallback } from 'react'
import type { Resource, ResourceInput, ResourceStatus, Summary } from '../types/resource'
import * as api from '../api/resources'
import { SummaryCards } from '../components/SummaryCards'
import { SearchBar } from '../components/SearchBar'
import { ResourceTable } from '../components/ResourceTable'
import { ResourceFormModal } from '../components/ResourceFormModal'

export function InventoryPage({ canManage }: { canManage: boolean }) {
  const [resources, setResources] = useState<Resource[]>([])
  const [summary, setSummary] = useState<Summary | null>(null)
  const [search, setSearch] = useState('')
  const [category, setCategory] = useState('')
  const [status, setStatus] = useState<ResourceStatus | ''>('')
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [modalTarget, setModalTarget] = useState<Resource | 'new' | null>(null)

  const refresh = useCallback(async () => {
    try {
      const [list, sum] = await Promise.all([
        api.listResources({
          search: search || undefined,
          category: category || undefined,
          status: status || undefined,
        }),
        api.getSummary(),
      ])
      setResources(list)
      setSummary(sum)
      setError(null)
    } catch (err) {
      setError(err instanceof Error ? err.message : '데이터를 불러오지 못했습니다.')
    } finally {
      setLoading(false)
    }
  }, [search, category, status])

  useEffect(() => {
    refresh()
  }, [refresh])

  async function handleAdjust(resource: Resource, delta: number) {
    try {
      await api.adjustQuantity(resource.id, delta)
      await refresh()
    } catch (err) {
      setError(err instanceof Error ? err.message : '수량 변경에 실패했습니다.')
    }
  }

  async function handleDelete(resource: Resource) {
    if (!confirm(`"${resource.name}"을(를) 삭제하시겠습니까?`)) return
    try {
      await api.deleteResource(resource.id)
      await refresh()
    } catch (err) {
      setError(err instanceof Error ? err.message : '삭제에 실패했습니다.')
    }
  }

  async function handleFormSubmit(data: ResourceInput) {
    if (modalTarget && modalTarget !== 'new') {
      await api.updateResource(modalTarget.id, data)
    } else {
      await api.createResource(data)
    }
    setModalTarget(null)
    await refresh()
  }

  const categories = Array.from(new Set(resources.map((r) => r.category).filter(Boolean)))

  return (
    <>
      {error && <div className="banner-error">{error}</div>}

      {summary && <SummaryCards summary={summary} />}

      <SearchBar
        search={search}
        onSearchChange={setSearch}
        category={category}
        onCategoryChange={setCategory}
        status={status}
        onStatusChange={setStatus}
        categories={categories}
        canAdd={canManage}
        onAddClick={() => setModalTarget('new')}
      />

      {loading ? (
        <p className="empty-state">불러오는 중...</p>
      ) : (
        <ResourceTable
          resources={resources}
          canManage={canManage}
          onAdjust={handleAdjust}
          onEdit={setModalTarget}
          onDelete={handleDelete}
        />
      )}

      {modalTarget && canManage && (
        <ResourceFormModal
          initial={modalTarget === 'new' ? undefined : modalTarget}
          onSubmit={handleFormSubmit}
          onClose={() => setModalTarget(null)}
        />
      )}
    </>
  )
}
