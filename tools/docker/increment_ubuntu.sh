#!/usr/bin/env bash
# This script updates everywhere the 'autonomy-ubuntu' image is used when a new version is available

# Get Today's Date
now="$(date +'%Y-%m-%d')"

# Update devcontainer.json
sed -i '' -E "s/autonomy-ubuntu:.*\",/autonomy-ubuntu:$now\",/" ../../.devcontainer/devcontainer.json

# Update codeql.yml
sed -i '' -E "s/autonomy-ubuntu:.*/autonomy-ubuntu:$now/" ../../.github/workflows/codeql.yml
