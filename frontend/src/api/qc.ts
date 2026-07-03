import type { Inspection, InspectionResult, QcSummary, QcTrendPoint } from '../types/qc'
import { request } from './client'

const BASE = '/api/qc'

export interface InspectionFilters {
  result?: InspectionResult
  product?: string
}

export function listInspections(filters: InspectionFilters = {}): Promise<Inspection[]> {
  const params = new URLSearchParams()
  if (filters.result) params.set('result', filters.result)
  if (filters.product) params.set('product', filters.product)
  const qs = params.toString()
  return request<Inspection[]>(`${BASE}${qs ? `?${qs}` : ''}`)
}

export function getQcSummary(): Promise<QcSummary> {
  return request<QcSummary>(`${BASE}/summary`)
}

export function getQcTrend(days = 7): Promise<QcTrendPoint[]> {
  return request<QcTrendPoint[]>(`${BASE}/trend?days=${days}`)
}
