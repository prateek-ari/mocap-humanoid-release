PYTHON ?= python3

.PHONY: hardware-free-checks lint typecheck test wire-check xsens-stream-check arm-safety-check syntax-check qualify frontend-check guide-check constraints

hardware-free-checks: syntax-check lint typecheck guide-check test wire-check xsens-stream-check arm-safety-check
	$(PYTHON) tools/mxtp02_replayer.py --synthetic --rate 120 --duration 0.1 \
		--with-joint-angles --quiet

guide-check:
	$(PYTHON) tools/docs/build_engineer_guide.py --check

# Run a captured G1 NPZ through the full qualification chain (window audit,
# conditioning, contact refinement, self-collision gate, controller screen,
# balance screen, npz->pkl conversion, PHUMA metrics + gate v2) and write one
# consolidated JSON ledger (ledger_version 3). See
# tools/qualify_g1_recording.py and docs/qualify-pipeline-2026-08-16.md.
#
#   make qualify RECORDING=path/to.npz [OFFSETS=path/to/offsets.json] [OUT_DIR=dir] [STAGE_TIMEOUT_S=seconds]
qualify:
	@if [ -z "$(RECORDING)" ]; then \
		echo "usage: make qualify RECORDING=path/to.npz [OFFSETS=path/to/offsets.json] [OUT_DIR=dir] [STAGE_TIMEOUT_S=seconds]"; \
		exit 2; \
	fi
	$(PYTHON) tools/qualify_g1_recording.py "$(RECORDING)" \
		$(if $(OFFSETS),--offsets "$(OFFSETS)") \
		$(if $(OUT_DIR),--out-dir "$(OUT_DIR)") \
		$(if $(STAGE_TIMEOUT_S),--stage-timeout-s "$(STAGE_TIMEOUT_S)")

syntax-check:
	$(PYTHON) -m compileall -q mocap_g1 eval tools tests
	git diff --check

lint:
	$(PYTHON) -m ruff check .

typecheck:
	$(PYTHON) -m mypy mocap_g1/safety.py mocap_g1/live_safety.py mocap_g1/arm_bridge.py

# pytest also collects module-level test functions, which unittest discover skips.
# CI passes PYTEST_ARGS="--cov=mocap_g1 --cov=eval ..." for report-only coverage.
test:
	$(PYTHON) -m pytest -q -rs $(PYTEST_ARGS)

# Regenerate the CI constraints (universal across Python 3.10-3.12). Requires uv.
# Review the diff and run the suite on 3.10, 3.11 and 3.12 before committing.
constraints:
	uv pip compile pyproject.toml --extra runtime --extra operator --extra test --extra lint \
		--universal --python-version 3.10 -o constraints/requirements-ci.txt

wire-check:
	$(PYTHON) tools/checks/decode_check.py --udp-loopback

xsens-stream-check:
	$(PYTHON) tools/xsens/stream_acceptance.py --self-test --require-joint-angles

arm-safety-check:
	$(PYTHON) tools/control/mock_g1_arm_bridge.py \
		--duration 1 --rate 50 --inject-stale-at 0.2

frontend-check:
	npm --prefix operator_ui ci
	npm --prefix operator_ui run typecheck
	npm --prefix operator_ui run lint
	npm --prefix operator_ui test
	npm --prefix operator_ui run build
	git diff --exit-code -- mocap_g1/operator/static
	@if [ -n "$$(git status --porcelain -- mocap_g1/operator/static)" ]; then \
		echo "ERROR: mocap_g1/operator/static is out of date: rebuilding the frontend changed or added files (including untracked ones). Commit the rebuilt bundle:"; \
		git status --porcelain -- mocap_g1/operator/static; \
		exit 1; \
	fi
