export interface InventoryLog {
  id: number
  resource_name: string
  action: 'create' | 'update' | 'adjust' | 'delete'
  detail: string
  username: string
  created_at: string
}
