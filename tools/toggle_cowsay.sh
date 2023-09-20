#!/usr/bin/env bash
# This script will toggle cowsay on or off.
toggle_command="/usr/games/fortune"
escaped_toggle_command=$(sed 's/[\/&]/\\&/g' <<< "$toggle_command")
bashrc_path="/root/.bashrc"

# Check if the command is already commented out in .bashrc.
if grep -F "# $toggle_command" "$bashrc_path"; then
    # Command is present and commented, so uncomment it.
    escaped_full_command=$(sed 's/[\/&]/\\&/g' <<< "$toggle_command")
    sed -i '/'"$escaped_full_command"'/s/^# //' $bashrc_path
    echo "Cowsay enabled..."
else
    # Command is either absent or uncommented, so comment it.
    escaped_full_command=$(sed 's/[\/&]/\\&/g' <<< "$toggle_command")
    sed -i '/'"$escaped_full_command"'/s/^/# /' $bashrc_path
    echo "Cowsay disabled..."
fi
