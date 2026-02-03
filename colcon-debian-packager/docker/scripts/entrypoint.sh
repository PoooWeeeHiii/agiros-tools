#!/bin/bash
# Entrypoint script for colcon-deb Docker container

set -e

# Source ROS setup if available
if [ -f "/opt/agiros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/agiros/${ROS_DISTRO}/setup.bash"
fi

# Handle user mapping for better file permissions
if [ -n "$LOCAL_USER_ID" ]; then
    echo "Starting with UID: $LOCAL_USER_ID"
    usermod -u $LOCAL_USER_ID builder
    chown -R builder:builder /home/builder
fi

# Setup environment
/usr/local/bin/setup-environment.sh

# Inject helper scripts if requested
if [ "$INJECT_HELPERS" = "true" ]; then
    /usr/local/bin/inject-helpers.sh
fi

# Execute the command
if [ $# -eq 0 ]; then
    # No command provided, start interactive shell
    exec /bin/bash
else
    # Execute provided command
    exec "$@"
fi