import type { Resource, ResourceInput, ResourceStatus, Summary } from '../types/resource'
import { request } from './client'

const BASE = '/api/resources'

export interface ListFilters {
  search?: string
  category?: string
  status?: ResourceStatus
}

export function listResources(filters: ListFilters = {}): Promise<Resource[]> {
  const params = new URLSearchParams()
  if (filters.search) params.set('search', filters.search)
  if (filters.category) params.set('category', filters.category)
  if (filters.status) params.set('status', filters.status)
  const qs = params.toString()
  return request<Resource[]>(`${BASE}${qs ? `?${qs}` : ''}`)
}

export function getSummary(): Promise<Summary> {
  return request<Summary>(`${BASE}/summary`)
}

export function createResource(data: ResourceInput): Promise<Resource> {
  return request<Resource>(BASE, { method: 'POST', body: JSON.stringify(data) })
}

export function updateResource(id: number, data: Partial<ResourceInput>): Promise<Resource> {
  return request<Resource>(`${BASE}/${id}`, { method: 'PUT', body: JSON.stringify(data) })
}

export function adjustQuantity(id: number, delta: number): Promise<Resource> {
  return request<Resource>(`${BASE}/${id}/quantity`, {
    method: 'PATCH',
    body: JSON.stringify({ delta }),
  })
}

export function deleteResource(id: number): Promise<void> {
  return request<void>(`${BASE}/${id}`, { method: 'DELETE' })
}
