export type InspectionResult = 'pass' | 'fail'

export interface Inspection {
  id: number
  product: string
  result: InspectionResult
  defect_location: string | null
  map_data: string | null
  created_at: string
}

export interface QcSummary {
  total: number
  passed: number
  failed: number
  defect_rate: number | null
}

export interface QcTrendPoint {
  date: string
  passed: number
  failed: number
}
