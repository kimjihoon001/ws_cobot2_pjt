import { useEffect, useState, useCallback } from 'react'
import type { Resource, ResourceInput, ResourceItemType, ResourceStatus, Summary } from '../types/resource'
import type { InventoryLog } from '../types/inventoryLog'
import * as api from '../api/resources'
import { SummaryCards } from '../components/SummaryCards'
import { SearchBar } from '../components/SearchBar'
import { ResourceTable } from '../components/ResourceTable'
import type { ResourceSortKey } from '../components/ResourceTable'
import { ResourceFormModal } from '../components/ResourceFormModal'
import { CategoryBarChart } from '../components/charts/CategoryBarChart'
import { Pagination } from '../components/Pagination'
import { SortableTh } from '../components/SortableTh'
import { downloadCsv } from '../utils/csv'
import { sortBy } from '../utils/sort'
import type { SortDir } from '../utils/sort'

type LogSortKey = 'created_at' | 'resource_name' | 'action' | 'username'

const PAGE_SIZE = 10

const ACTION_LABEL: Record<InventoryLog['action'], string> = {
  create: '등록',
  update: '수정',
  adjust: '수량변경',
  delete: '삭제',
}

export function InventoryPage({ canManage }: { canManage: boolean }) {
  const [view, setView] = useState<'list' | 'history'>('list')
  const [resources, setResources] = useState<Resource[]>([])
  const [summary, setSummary] = useState<Summary | null>(null)
  const [search, setSearch] = useState('')
  const [category, setCategory] = useState('')
  const [itemType, setItemType] = useState<ResourceItemType | ''>('')
  const [status, setStatus] = useState<ResourceStatus | ''>('')
  const [page, setPage] = useState(1)
  const [sortKey, setSortKey] = useState<ResourceSortKey>('id')
  const [sortDir, setSortDir] = useState<SortDir>('asc')
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [modalTarget, setModalTarget] = useState<Resource | 'new' | null>(null)
  const [logs, setLogs] = useState<InventoryLog[]>([])
  const [logsLoading, setLogsLoading] = useState(false)
  const [logSortKey, setLogSortKey] = useState<LogSortKey>('created_at')
  const [logSortDir, setLogSortDir] = useState<SortDir>('desc')

  const refresh = useCallback(async () => {
    try {
      const [list, sum] = await Promise.all([
        api.listResources({
          search: search || undefined,
          category: category || undefined,
          item_type: itemType || undefined,
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
  }, [search, category, itemType, status])

  useEffect(() => {
    refresh()
  }, [refresh])

  useEffect(() => {
    setPage(1)
  }, [search, category, itemType, status, sortKey, sortDir])

  function handleSort(key: ResourceSortKey) {
    if (key === sortKey) {
      setSortDir((d) => (d === 'asc' ? 'desc' : 'asc'))
    } else {
      setSortKey(key)
      setSortDir('asc')
    }
  }

  function handleLogSort(key: LogSortKey) {
    if (key === logSortKey) {
      setLogSortDir((d) => (d === 'asc' ? 'desc' : 'asc'))
    } else {
      setLogSortKey(key)
      setLogSortDir('asc')
    }
  }

  useEffect(() => {
    if (view !== 'history' || !canManage) return
    setLogsLoading(true)
    api
      .listInventoryLogs()
      .then(setLogs)
      .catch(() => setLogs([]))
      .finally(() => setLogsLoading(false))
  }, [view, canManage])

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

  function handleExport() {
    downloadCsv(
      `inventory_${new Date().toISOString().slice(0, 10)}.csv`,
      ['ID', '이름', '구분', '분류', '위치', '수량', '단위', '최소수량', '상태'],
      resources.map((r) => [
        r.id,
        r.name,
        r.item_type === 'product' ? '완제품' : '원자재',
        r.category,
        r.location,
        r.quantity,
        r.unit,
        r.min_quantity,
        r.status,
      ]),
    )
  }

  const categories = Array.from(new Set(resources.map((r) => r.category).filter(Boolean)))

  const orderMap: Record<string, number> = {
    '기본형': 1,
    'A형': 2,
    'B형': 3,
    'C형': 4
  }

  const categoryChartData = categories
    .map((cat) => ({
      label: cat,
      value: resources
        .filter((r) => r.category === cat)
        .reduce((sum, r) => sum + r.quantity, 0),
    }))
    .sort((a, b) => {
      const orderA = orderMap[a.label] || 99
      const orderB = orderMap[b.label] || 99
      if (orderA === 99 && orderB === 99) return a.label.localeCompare(b.label)
      return orderA - orderB
    })

  const sortedResources = sortBy(resources, sortKey, sortDir)
  const pagedResources = sortedResources.slice((page - 1) * PAGE_SIZE, page * PAGE_SIZE)
  const sortedLogs = sortBy(logs, logSortKey, logSortDir)

  return (
    <>
      {canManage && (
        <nav className="tabs">
          <button
            type="button"
            className={view === 'list' ? 'tab tab-active' : 'tab'}
            onClick={() => setView('list')}
          >
            재고 현황
          </button>
          <button
            type="button"
            className={view === 'history' ? 'tab tab-active' : 'tab'}
            onClick={() => setView('history')}
          >
            변경 이력
          </button>
        </nav>
      )}

      {view === 'history' && canManage ? (
        logsLoading ? (
          <p className="empty-state">불러오는 중...</p>
        ) : (
          <table className="resource-table">
            <thead>
              <tr>
                <SortableTh label="일시" sortKey="created_at" activeKey={logSortKey} dir={logSortDir} onSort={handleLogSort} />
                <SortableTh label="자원" sortKey="resource_name" activeKey={logSortKey} dir={logSortDir} onSort={handleLogSort} />
                <SortableTh label="작업" sortKey="action" activeKey={logSortKey} dir={logSortDir} onSort={handleLogSort} />
                <th>내용</th>
                <SortableTh label="처리자" sortKey="username" activeKey={logSortKey} dir={logSortDir} onSort={handleLogSort} />
              </tr>
            </thead>
            <tbody>
              {sortedLogs.length === 0 ? (
                <tr>
                  <td colSpan={5} className="empty-state">
                    변경 이력이 없습니다.
                  </td>
                </tr>
              ) : (
                sortedLogs.map((log) => (
                  <tr key={log.id}>
                    <td>{new Date(log.created_at).toLocaleString()}</td>
                    <td>{log.resource_name}</td>
                    <td>{ACTION_LABEL[log.action]}</td>
                    <td>{log.detail}</td>
                    <td>{log.username}</td>
                  </tr>
                ))
              )}
            </tbody>
          </table>
        )
      ) : (
        <>
          {error && <div className="banner-error">{error}</div>}

          {summary && <SummaryCards summary={summary} />}

          {!loading && <CategoryBarChart title="카테고리별 재고 수량" data={categoryChartData} />}

          <SearchBar
            search={search}
            onSearchChange={setSearch}
            category={category}
            onCategoryChange={setCategory}
            itemType={itemType}
            onItemTypeChange={setItemType}
            status={status}
            onStatusChange={setStatus}
            categories={categories}
            canAdd={canManage}
            onAddClick={() => setModalTarget('new')}
            onExportClick={resources.length > 0 ? handleExport : undefined}
          />

          {loading ? (
            <p className="empty-state">불러오는 중...</p>
          ) : (
            <>
              <ResourceTable
                resources={pagedResources}
                canManage={canManage}
                sortKey={sortKey}
                sortDir={sortDir}
                onSort={handleSort}
                onAdjust={handleAdjust}
                onEdit={setModalTarget}
                onDelete={handleDelete}
              />
              <Pagination
                page={page}
                pageSize={PAGE_SIZE}
                total={resources.length}
                onPageChange={setPage}
              />
            </>
          )}

          {modalTarget && canManage && (
            <ResourceFormModal
              initial={modalTarget === 'new' ? undefined : modalTarget}
              onSubmit={handleFormSubmit}
              onClose={() => setModalTarget(null)}
            />
          )}
        </>
      )}
    </>
  )
}
