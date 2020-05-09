#!/usr/bin/env bash

# Usage:
#   clang-format.sh </some/path>

TARGET=$1

# Check tool.
if [ ! -x "$(command -v clang-format)" ]; then
  echo "Installing clang-format..."
  sudo apt-get install -y clang-format
fi

# Format.
if [ -f "${TARGET}" ]; then
  clang-format -i -style=Google "${TARGET}"
else
  clang-format -i -style=file \
	$(find "${TARGET}" -type f | grep -e '\.h$' -e '\.hpp$' -e '\.cc$' -e '\.cpp$')
fi
