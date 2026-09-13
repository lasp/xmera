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

# Stage one: which executables carry the fuzz label.
FUZZ_BINS=$(BUILD_DIR="$BUILD_DIR" python3 - <<'PY'
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

# Stage two, for each executable: which FUZZ_TESTs it contains. The CTest listing
# cannot give this. A fuzz binary that includes its unit-test translation unit gives
# those unit tests the same label. If you give a non-fuzz name to --fuzz, the binary
# prints "No FUZZ_TEST matches the name" and stops with a non-zero exit code.
failures=()

while IFS= read -r fuzz_bin; do
  [[ -z "$fuzz_bin" ]] && continue
  fuzz_name="$(basename "$fuzz_bin")"

  fuzz_tests=$("$fuzz_bin" --list_fuzz_tests </dev/null | sed -n 's/^\[\*\] Fuzz test: //p')
  if [[ -z "$fuzz_tests" ]]; then
    echo "No FUZZ_TESTs registered in ${fuzz_name}, but it carries the fuzz label." >&2
    exit 1
  fi

  while IFS= read -r test_name; do
    [[ -z "$test_name" ]] && continue
    echo ">>> Running long fuzz session for ${fuzz_name} / ${test_name}"

    # One corpus directory for each fuzz test. Each FUZZ_TEST serializes its own
    # parameter signature. Thus if two tests use the same directory, each test rejects
    # the files of the other test with "Unexpected intermediate representation".
    if [[ -n "$CORPUS_DIR" ]]; then
      test_corpus_dir="${CORPUS_DIR}/${fuzz_name}/${test_name}"
      mkdir -p "$test_corpus_dir"
      export FUZZTEST_TESTSUITE_IN_DIR="$test_corpus_dir"
      export FUZZTEST_TESTSUITE_OUT_DIR="$test_corpus_dir"
    fi

    set +e
    "$fuzz_bin" --fuzz="$test_name" --fuzz_for="$FUZZ_DURATION" </dev/null 2>&1 \
      | tee "${LOG_DIR}/${fuzz_name}.${test_name}.log"
    status=${PIPESTATUS[0]}
    set -e

    if [[ $status -ne 0 ]]; then
      failures+=("${fuzz_name} / ${test_name} (exit ${status})")
    fi
  done <<< "$fuzz_tests"
done <<< "$FUZZ_BINS"

if [[ ${#failures[@]} -gt 0 ]]; then
  echo
  echo "Fuzz targets with findings:"
  for failure in "${failures[@]}"; do
    echo "  ${failure}"
  done
  exit 1
fi

echo
echo "All fuzz targets completed without findings."
