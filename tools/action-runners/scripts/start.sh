#!/bin/bash

# Ensure required environment variables are set
if [[ -z "$GH_OWNER" || -z "$GH_REPOSITORY" || -z "$GH_TOKEN" ]]; then
    echo "Error: GH_OWNER, GH_REPOSITORY, and GH_TOKEN must be set as environment variables."
    exit 1
fi

# Generate a unique runner name
RUNNER_SUFFIX=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 5 | head -n 1)
RUNNER_NAME="Autonomy-${RUNNER_SUFFIX}"

# Fetch the registration token from GitHub
REG_TOKEN=$(curl -sX POST \
    -H "Accept: application/vnd.github.v3+json" \
    -H "Authorization: token ${GH_TOKEN}" \
    https://api.github.com/repos/${GH_OWNER}/${GH_REPOSITORY}/actions/runners/registration-token | jq .token --raw-output)


# Check if the registration token was retrieved successfully
if [[ -z "$REG_TOKEN" || "$REG_TOKEN" == "null" ]]; then
    echo "Error: Failed to retrieve registration token. Check your GH_TOKEN and repository details."
    exit 1
fi

# Change to the actions-runner directory
cd /home/docker/actions-runner || { echo "Error: actions-runner directory not found."; exit 1; }

# Configure the runner
./config.sh --unattended \
    --url "https://github.com/${GH_OWNER}/${GH_REPOSITORY}" \
    --token "${REG_TOKEN}" \
    --name "${RUNNER_NAME}"

# Cleanup function to remove the runner during shutdown
cleanup() {
    echo "Removing runner..."
    ./config.sh remove --unattended --token "${REG_TOKEN}"
}

# Trap signals for proper cleanup
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# Start the runner
./run.sh & wait $!