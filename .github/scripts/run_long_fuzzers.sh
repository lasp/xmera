#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <build_dir> <log_dir> [--fuzz-for <duration>] [--corpus <path>]" >&2
  exit 2
fi

BUILD_DIR="$1"
LOG_DIR="$2"
shift 2

FUZZ_DURATION="30s"
CORPUS_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fuzz-for)
      [[ $# -ge 2 ]] || { echo "--fuzz-for requires a value" >&2; exit 2; }
      FUZZ_DURATION="$2"
      shift 2
      ;;
    --corpus)
      [[ $# -ge 2 ]] || { echo "--corpus requires a path" >&2; exit 2; }
      CORPUS_DIR="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

mkdir -p "$LOG_DIR"
if [[ -n "$CORPUS_DIR" ]]; then
  mkdir -p "$CORPUS_DIR"
fi

FUZZ_BINS=$(BUILD_DIR="$BUILD_DIR" python - <<'PY'
import json, os, subprocess
build_dir = os.environ["BUILD_DIR"]
proc = subprocess.run(
    ["ctest", "--show-only=json-v1", "-L", "fuzz"],
    capture_output=True, text=True, check=True,
    cwd=build_dir
)
data = json.loads(proc.stdout or "{}")
seen = []
for test in data.get("tests", []):
    cmd = test.get("command", [])
    if cmd:
        exe = cmd[0]
        if exe not in seen:
            seen.append(exe)
print("\n".join(seen))
PY
)
if [[ -z "$FUZZ_BINS" ]]; then
  echo "No fuzz executables discovered via CTest labels." >&2
  exit 1
fi
while IFS= read -r fuzz_bin; do
  [[ -z "$fuzz_bin" ]] && continue
  fuzz_name="$(basename "$fuzz_bin")"
  echo ">>> Running long fuzz session for ${fuzz_name}"
  cmd=("$fuzz_bin")
  if [[ -n "$FUZZ_DURATION" ]]; then
    cmd+=("--fuzz_for=${FUZZ_DURATION}")
  fi
  env_vars=()
  if [[ -n "$CORPUS_DIR" ]]; then
    fuzz_corpus_dir="${CORPUS_DIR}/${fuzz_name}"
    mkdir -p "$fuzz_corpus_dir"
    env_vars+=(
      "FUZZTEST_TESTSUITE_OUT_DIR=${fuzz_corpus_dir}"
      "FUZZTEST_TESTSUITE_IN_DIR=${fuzz_corpus_dir}"
    )
  fi
  env "${env_vars[@]}" "${cmd[@]}" 2>&1 | tee "$LOG_DIR/${fuzz_name}.log" || true
done <<< "$FUZZ_BINS"
