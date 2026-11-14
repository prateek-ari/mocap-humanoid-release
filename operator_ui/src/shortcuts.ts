/** DISENGAGE keyboard shortcut.  Escape never activates a focused routine key. */
export const DISENGAGE_SHORTCUT_KEY = 'Escape'
export const DISENGAGE_SHORTCUT_LABEL = 'Esc'

const NON_TEXT_INPUT_TYPES = new Set(['button', 'checkbox', 'color', 'file', 'image', 'radio', 'range', 'reset', 'submit'])

/** True when keyboard input at ``target`` edits text (inputs, textareas, selects, contenteditable). */
export function isTextEntryTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false
  if (target.isContentEditable) return true
  if (target instanceof HTMLTextAreaElement || target instanceof HTMLSelectElement) return true
  if (target instanceof HTMLInputElement) return !NON_TEXT_INPUT_TYPES.has(target.type)
  return false
}

export function isDisengageShortcut(event: KeyboardEvent): boolean {
  return event.key === DISENGAGE_SHORTCUT_KEY
    && !event.repeat
    && !event.isComposing
    && !event.defaultPrevented
    && !event.ctrlKey && !event.altKey && !event.metaKey
    && !isTextEntryTarget(event.target)
}
