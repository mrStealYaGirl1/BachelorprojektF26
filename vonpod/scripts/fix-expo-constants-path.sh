#!/usr/bin/env bash

set -euo pipefail

TARGET="node_modules/expo-constants/scripts/get-app-config-ios.sh"

if [ ! -f "$TARGET" ]; then
  echo "[fix-expo-constants-path] Skipping: $TARGET not found"
  exit 0
fi

# Expo Constants script breaks when project paths contain spaces.
# Ensure PROJECT_DIR is quoted before basename expansion.
if grep -q 'PROJECT_DIR_BASENAME=$(basename \$PROJECT_DIR)' "$TARGET"; then
  sed -i '' 's/PROJECT_DIR_BASENAME=$(basename \$PROJECT_DIR)/PROJECT_DIR_BASENAME=$(basename "$PROJECT_DIR")/' "$TARGET"
  echo "[fix-expo-constants-path] Patched $TARGET"
else
  echo "[fix-expo-constants-path] No patch needed"
fi
