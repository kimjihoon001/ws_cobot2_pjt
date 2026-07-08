export type SortDir = 'asc' | 'desc'

export function sortBy<T, K extends keyof T>(list: T[], key: K, dir: SortDir): T[] {
  return [...list].sort((a, b) => {
    let av = a[key]
    let bv = b[key]
    if (typeof av === 'string') av = av.toLowerCase() as T[K]
    if (typeof bv === 'string') bv = bv.toLowerCase() as T[K]
    if (av < bv) return dir === 'asc' ? -1 : 1
    if (av > bv) return dir === 'asc' ? 1 : -1
    return 0
  })
}
