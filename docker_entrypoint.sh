#!/bin/bash
set -e

# setup ros environment
source "/usr/src/ambf/build/devel/setup.bash"
exec "$@"