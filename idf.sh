#!/usr/bin/env bash
#
# Convenience wrapper for ESP-IDF.
#
# ESP-IDF is not on the PATH in this environment, so this script sources the
# installed toolchain before delegating to idf.py. Every idf.py argument is
# forwarded unchanged, e.g.:
#
#   ./idf.sh build
#   ./idf.sh -p /dev/ttyACM0 flash
#   ./idf.sh -p /dev/ttyACM0 monitor
#
# Override the toolchain location with IDF_PATH if you move things around.

set -euo pipefail

IDF_PATH_DEFAULT="/home/nuc/github/esp-hosted/esp_hosted_ng/esp/esp_driver/esp-idf"

if [[ -z "${IDF_PATH:-}" ]]; then
    if [[ ! -f "${IDF_PATH_DEFAULT}/export.sh" ]]; then
        echo "error: ESP-IDF not found at ${IDF_PATH_DEFAULT}" >&2
        echo "       set IDF_PATH, or edit IDF_PATH_DEFAULT in $0" >&2
        exit 1
    fi
    # shellcheck disable=SC1091
    source "${IDF_PATH_DEFAULT}/export.sh" >/dev/null
fi

exec idf.py "$@"
