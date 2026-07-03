export type ResourceStatus = 'normal' | 'low' | 'out'
export type ResourceItemType = 'material' | 'product'

export interface Resource {
  id: number
  name: string
  item_type: ResourceItemType
  category: string
  quantity: number
  unit: string
  min_quantity: number
  location: string
  status: ResourceStatus
  created_at: string
  updated_at: string
}

export interface ResourceInput {
  name: string
  item_type: ResourceItemType
  category: string
  quantity: number
  unit: string
  min_quantity: number
  location: string
}

export interface Summary {
  total: number
  normal: number
  low: number
  out: number
}
