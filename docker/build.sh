#!/bin/bash

# Key-value pairs to update
USERNAME=$(id -un)
USER_UID=$(id -u)
USER_GID=$(id -g)

# Update or append variables in .env
update_env_var() {
    local key="$1"
    local value="$2"
    if grep -q "^${key}=" .env; then
        sed -i "s/^${key}=.*/${key}=${value}/" .env
    else
        echo "${key}=${value}" >> .env
    fi
}

# Ensure .env exists
touch .env

# Inject user info
update_env_var USERNAME "$USERNAME"
update_env_var USER_UID "$USER_UID"
update_env_var USER_GID "$USER_GID"

# Start the container
docker compose up --build
