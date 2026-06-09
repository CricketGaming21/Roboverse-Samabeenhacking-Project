.PHONY: test gate lint fmt
PY ?= python
test:            ## full regression (fake-SDK substrate, no network, deterministic)
	$(PY) -m pytest
gate-%:          ## run one phase gate then the full regression: make gate-p3
	$(PY) -m pytest -m $* && $(PY) -m pytest
