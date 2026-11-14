// typescript-eslint parses with the TypeScript JS API, which TypeScript 7 no
// longer ships.  This local package gives it a private TypeScript 6 copy; the
// project's `typecheck` still runs the TypeScript 7 compiler.
export { default } from 'typescript-eslint'
