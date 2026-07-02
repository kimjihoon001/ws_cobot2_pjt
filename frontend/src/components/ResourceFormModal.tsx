import { useState } from 'react'
import type { FormEvent } from 'react'
import type { Resource, ResourceInput } from '../types/resource'

interface Props {
  initial?: Resource
  onSubmit: (data: ResourceInput) => Promise<void>
  onClose: () => void
}

const EMPTY: ResourceInput = {
  name: '',
  category: '',
  quantity: 0,
  unit: 'EA',
  min_quantity: 0,
  location: '',
}

export function ResourceFormModal({ initial, onSubmit, onClose }: Props) {
  const [form, setForm] = useState<ResourceInput>(
    initial
      ? {
          name: initial.name,
          category: initial.category,
          quantity: initial.quantity,
          unit: initial.unit,
          min_quantity: initial.min_quantity,
          location: initial.location,
        }
      : EMPTY,
  )
  const [error, setError] = useState<string | null>(null)
  const [submitting, setSubmitting] = useState(false)

  async function handleSubmit(e: FormEvent) {
    e.preventDefault()
    if (!form.name.trim()) {
      setError('이름을 입력하세요.')
      return
    }
    setSubmitting(true)
    setError(null)
    try {
      await onSubmit(form)
    } catch (err) {
      setError(err instanceof Error ? err.message : '저장에 실패했습니다.')
    } finally {
      setSubmitting(false)
    }
  }

  return (
    <div className="modal-backdrop" onClick={onClose}>
      <form className="modal" onClick={(e) => e.stopPropagation()} onSubmit={handleSubmit}>
        <h2>{initial ? '자원 수정' : '자원 추가'}</h2>

        <label>
          이름
          <input
            value={form.name}
            onChange={(e) => setForm({ ...form, name: e.target.value })}
            autoFocus
          />
        </label>

        <label>
          분류
          <input
            value={form.category}
            onChange={(e) => setForm({ ...form, category: e.target.value })}
            placeholder="예: 공구, 자재"
          />
        </label>

        <div className="form-row">
          <label>
            수량
            <input
              type="number"
              min={0}
              value={form.quantity}
              onChange={(e) => setForm({ ...form, quantity: Number(e.target.value) })}
            />
          </label>
          <label>
            단위
            <input
              value={form.unit}
              onChange={(e) => setForm({ ...form, unit: e.target.value })}
              placeholder="EA"
            />
          </label>
        </div>

        <label>
          최소수량 (부족 판정 기준)
          <input
            type="number"
            min={0}
            value={form.min_quantity}
            onChange={(e) => setForm({ ...form, min_quantity: Number(e.target.value) })}
          />
        </label>

        <label>
          위치
          <input
            value={form.location}
            onChange={(e) => setForm({ ...form, location: e.target.value })}
            placeholder="예: A-01"
          />
        </label>

        {error && <p className="form-error">{error}</p>}

        <div className="modal-actions">
          <button type="button" className="text-btn" onClick={onClose}>
            취소
          </button>
          <button type="submit" className="primary-btn" disabled={submitting}>
            {submitting ? '저장 중...' : '저장'}
          </button>
        </div>
      </form>
    </div>
  )
}
