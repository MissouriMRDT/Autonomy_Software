#!/usr/bin/env bash
# This script updates everywhere the 'autonomy-jetpack' image is used when a new version is available

# Get Today's Date
now="$(date +'%Y-%m-%d')"

# Update devcontainer.json
sed -i '' -E "s/autonomy-jetpack:.*\",/autonomy-jetpack:$now\",/" ../../.devcontainer/devcontainer.json
