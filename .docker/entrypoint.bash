set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash"
source "/dev_ws/devel/setup.bash"

exec "$@"