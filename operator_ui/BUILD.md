# Building the Setup console UI

This folder is the React source of the bundle in `mocap_g1/operator/static/`.
Rebuild the bundle after every change here and commit both together.

```bash
cd operator_ui
npm ci            # Node 22 (see .nvmrc), npm 10; uses package-lock.json
npm run typecheck && npm run lint && npm test
npm run build     # writes ../mocap_g1/operator/static (empties it first)
```

The build is reproducible: the same source and lockfile give byte-identical
files in `mocap_g1/operator/static/`. Check with `git status` after a build
of unchanged source; nothing in `static/` should change.

Do not commit `node_modules/`.
