

PROJECT = lab4
SRC_DIR = src
MAIN    = main.py
VENV    = .venv
STUBS   = micropython-rp2-stubs

MPREMOTE = mpremote

# Every .py under SRC_DIR (recursive), as paths relative to SRC_DIR.
# Hidden directories (.venv, .git, .idea) are skipped.
AUTO_SOURCES = $(shell find $(SRC_DIR) -type f -name "*.py" -not -path "*/.*" -printf "%P\n" | sort)

# Sub-directories that must exist on the Pico before copying (parents first).
AUTO_DIRS = $(shell find $(SRC_DIR) -mindepth 1 -type d -not -name "__pycache__" -not -path "*/.*" -printf "%P\n" | sort)

.DEFAULT_GOAL := help

.PHONY: help setup run deploy once wipe repl ls reset devices print clean test hw

help:
	@awk 'BEGIN {FS = ":.*##"; printf "\nAvailable commands:\n"} \
	/^[a-zA-Z_-]+:.*##/ { printf "  %-15s %s\n", $$1, $$2 }' $(MAKEFILE_LIST)

## Environment

setup: $(VENV) ## create venv and install stubs (code completion)

$(VENV):
	python3 -m venv $(VENV)
	$(VENV)/bin/pip install -U $(STUBS)

## Running

run: ## run main on the Pico from local files (nothing copied)
	$(MPREMOTE) mount $(SRC_DIR) run $(SRC_DIR)/$(MAIN)

deploy: ## copy all sources to the Pico and reset (runs on boot)
	@for d in $(AUTO_DIRS); do $(MPREMOTE) mkdir :$$d >/dev/null 2>&1 || true; done
	$(MPREMOTE) $(foreach f,$(AUTO_SOURCES),cp $(SRC_DIR)/$(f) :$(f) +) reset

once: wipe deploy ## your best friend: wipe, deploy, reset

## Testing

test: ## run the logic and simulation tests on the PC (no robot needed)
	python3 tests/host/test_follower.py
	python3 tests/host/test_robot_sim.py

hw: ## on-device checks: make hw-motors, hw-sensors, hw-sonar or hw-calibrate
	@test -n "$(T)" || { echo "Use: make hw-motors | make hw-sensors | make hw-sonar | make hw-calibrate"; exit 1; }
	$(MPREMOTE) mount $(SRC_DIR) run tests/hw/$(T).py

hw-%:
	$(MPREMOTE) mount $(SRC_DIR) run tests/hw/$*.py

## Device actions

wipe: ## delete every file on the Pico
	$(MPREMOTE) rm -r :

ls: ## list files on the Pico
	$(MPREMOTE) ls

reset: ## reset the Pico
	$(MPREMOTE) reset

devices: ## list connected serial devices
	$(MPREMOTE) connect list

## Housekeeping

print: ## show discovered sources and directories
	@echo "Sources: $(AUTO_SOURCES)"
	@echo "Dirs:    $(AUTO_DIRS)"

clean: ## delete venv and Python caches
	rm -rf $(VENV)
	find . -type d -name "__pycache__" -not -path "./.venv/*" -exec rm -rf {} +
