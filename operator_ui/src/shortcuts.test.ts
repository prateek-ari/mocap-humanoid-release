// @vitest-environment jsdom
import { describe, expect, it } from 'vitest'
import { isDisengageShortcut, isTextEntryTarget } from './shortcuts'

function element(html: string): HTMLElement {
  const host = document.createElement('div')
  host.innerHTML = html
  return host.firstElementChild as HTMLElement
}

describe('isTextEntryTarget', () => {
  it('treats text-editing controls as text entry', () => {
    for (const html of ['<input>', '<input type="number">', '<input type="search">', '<textarea></textarea>', '<select></select>']) {
      expect(isTextEntryTarget(element(html)), html).toBe(true)
    }
    const editable = element('<div contenteditable="true"></div>')
    Object.defineProperty(editable, 'isContentEditable', { value: true })
    expect(isTextEntryTarget(editable)).toBe(true)
  })

  it('does not treat buttons, checkboxes or the page as text entry', () => {
    for (const html of ['<button></button>', '<input type="checkbox">', '<input type="radio">', '<div></div>']) {
      expect(isTextEntryTarget(element(html)), html).toBe(false)
    }
    expect(isTextEntryTarget(null)).toBe(false)
    expect(isTextEntryTarget(window)).toBe(false)
  })
})

describe('isDisengageShortcut', () => {
  const press = (init: KeyboardEventInit, target: EventTarget = document.body) => {
    let matched = false
    const listener = (event: Event) => { matched = isDisengageShortcut(event as KeyboardEvent) }
    target.addEventListener('keydown', listener)
    target.dispatchEvent(new KeyboardEvent('keydown', { bubbles: true, cancelable: true, ...init }))
    target.removeEventListener('keydown', listener)
    return matched
  }

  it('matches a plain Escape press only', () => {
    expect(press({ key: 'Escape' })).toBe(true)
    expect(press({ key: 'Escape' }, element('<input type="checkbox">'))).toBe(true)
    expect(press({ key: ' ' })).toBe(false)
    expect(press({ key: 'Escape', shiftKey: false, altKey: true })).toBe(false)
    expect(press({ key: 'Escape', metaKey: true })).toBe(false)
    expect(press({ key: 'Escape', repeat: true })).toBe(false)
    expect(press({ key: 'Escape' }, element('<input>'))).toBe(false)
  })
})
