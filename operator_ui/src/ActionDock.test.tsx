// @vitest-environment jsdom
import { cleanup, render, screen, within } from '@testing-library/react'
import userEvent from '@testing-library/user-event'
import { afterEach, describe, expect, it, vi } from 'vitest'
import { ActionDock, KeyBank } from './ActionDock'
import { gateFor as computeGate } from './gating'
import { makeStatus } from './test/fixtures'
import type { ActionName, OperatorStatus } from './types'

afterEach(cleanup)

const ROUTINE_KEYS = ['Check', 'Mocap', 'Simulate', 'Robot', 'Preflight', 'Arm', 'Track', 'Rearm']

interface DockSetup {
  status?: OperatorStatus
  pending?: ActionName[]
  wsConnected?: boolean
}

function renderDock({ status = makeStatus(), pending = [], wsConnected = true }: DockSetup = {}) {
  const onAction = vi.fn()
  const pendingSet: ReadonlySet<ActionName> = new Set(pending)
  const gateFor = (name: ActionName) => computeGate(name, { status, wsConnected, preflightComplete: true })
  render(
    <ActionDock
      status={status}
      pending={pendingSet}
      preflightComplete
      gateFor={gateFor}
      inspect={null}
      onAction={onAction}
      keyBank={<KeyBank status={status} pending={pendingSet} gateFor={gateFor} onAction={onAction} onInspect={() => {}} idPrefix="dock" />}
    />,
  )
  const dock = screen.getByRole('contentinfo', { name: 'Session actions' })
  const stop = within(dock).getByRole('button', { name: /^Disengage\./ })
  const bankKey = (label: string) => {
    const key = within(dock).getAllByRole('button').find((button) => (
      button.classList.contains('gate-key') && !button.classList.contains('next-key')
      && button.querySelector('.key-label')?.textContent === label
    ))
    if (!key) throw new Error(`no key labelled ${label}`)
    return key
  }
  return { onAction, stop, bankKey, user: userEvent.setup() }
}

function faultStatus(): OperatorStatus {
  const base = makeStatus({
    state: 'fault_latched',
    active_fault: { code: 'health_stale', reason: 'Runtime health is stale', source: 'controller', timestamp: 1, latched: true, recoverable: true },
  })
  const controls = Object.fromEntries(Object.entries(base.controls).map(([name, control]) => [
    name,
    name === 'disengage' || name === 'shutdown' ? control : { enabled: false, reason: 'Requires idle' },
  ]))
  return { ...base, controls }
}

describe.each<[string, DockSetup]>([
  ['a routine request is pending', { pending: ['start-tracking'] }],
  ['the event stream is lost', { wsConnected: false }],
  ['a fault is latched', { status: faultStatus() }],
])('while %s', (_label, setup) => {
  it('keeps DISENGAGE enabled, focusable and clickable', async () => {
    const { onAction, stop, user } = renderDock(setup)

    expect(stop.getAttribute('aria-disabled')).toBe('false')
    await user.click(stop)
    expect(onAction).toHaveBeenCalledWith('disengage')

    onAction.mockClear()
    stop.focus()
    await user.keyboard('{Enter}')
    expect(onAction).toHaveBeenCalledWith('disengage')
  })

  it('locks every routine key but leaves it focusable with its reason', async () => {
    const { onAction, bankKey, user } = renderDock(setup)

    for (const label of ROUTINE_KEYS) {
      const key = bankKey(label)
      expect(key.getAttribute('aria-disabled'), label).toBe('true')
      expect(key.hasAttribute('disabled'), label).toBe(false)
      const reasonId = key.getAttribute('aria-describedby')
      expect(reasonId && document.getElementById(reasonId)?.textContent, label).toBeTruthy()
      await user.click(key)
    }
    expect(onAction).not.toHaveBeenCalled()
  })
})

describe('DISENGAGE double submit', () => {
  it('ignores a second press while its own request is pending', async () => {
    const { onAction, stop, user } = renderDock({ pending: ['disengage'] })
    expect(stop.getAttribute('aria-disabled')).toBe('true')
    await user.click(stop)
    expect(onAction).not.toHaveBeenCalled()
  })

  it('stays clickable while End session is pending', async () => {
    const { onAction, stop, user } = renderDock({ pending: ['shutdown'] })
    await user.click(stop)
    expect(onAction).toHaveBeenCalledWith('disengage')
  })
})

it('documents the DISENGAGE shortcut on the key', () => {
  const { stop } = renderDock()
  expect(stop.getAttribute('aria-keyshortcuts')).toBe('Escape')
  expect(screen.getByText('Esc').tagName).toBe('KBD')
})
