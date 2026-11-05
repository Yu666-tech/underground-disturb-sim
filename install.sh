#!/usr/bin/env bash
# Install a self-contained source delivery into a dedicated workspace.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NO_SHELL=false
if [[ "${1:-}" == --no-shell ]]; then NO_SHELL=true; shift; fi
[[ $# == 0 ]] || { echo 'Usage: bash install.sh [--no-shell]'; exit 2; }
[[ -f /etc/os-release ]] || exit 1
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo 'Requires Ubuntu 22.04'; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ROS2 Humble not found'; exit 1; }
[[ "$(id -u)" != 0 ]] || { echo 'Run as your normal Ubuntu user, not sudo'; exit 1; }
TARGET="${COAL_INSTALL_ROOT:-$HOME/coal_lidar_ws_20261105}"
TARGET="$(realpath -m -- "$TARGET")"
[[ "$TARGET" != / && "$TARGET" != "$HOME" && "$TARGET" != "$ROOT" ]] || { echo 'Unsafe target directory'; exit 1; }
if [[ -d "$TARGET" && -n "$(ls -A -- "$TARGET")" && ! -f "$TARGET/.coal_lidar_delivery.json" ]]; then
  echo "Target is not an installer-managed workspace: $TARGET"; exit 1
fi
python3 "$ROOT/scripts/verify_bundle.py" "$ROOT"
packages=(build-essential cmake python3-colcon-common-extensions python3-numpy python3-yaml
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev ros-humble-rviz2 ros-humble-tf2-ros
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime)
missing=()
for package in "${packages[@]}"; do
  if [[ "$(dpkg-query -W -f='${db:Status-Abbrev}' "$package" 2>/dev/null || true)" != ii* ]]; then missing+=("$package"); fi
done
if (( ${#missing[@]} )); then
  echo "Installing missing dependencies: ${missing[*]}"
  sudo apt-get update
  sudo apt-get install -y "${missing[@]}"
fi
# Gazebo Classic 11.10.2 prints its version but may return 255 for --version.
gazebo_version="$(gazebo --version 2>&1 || true)"
grep -E 'version 11\.' <<< "$gazebo_version" >/dev/null || { echo 'Gazebo Classic 11 required'; exit 1; }
mkdir -p -- "$TARGET"
cp -- "$ROOT/RELEASE.json" "$TARGET/.coal_lidar_delivery.json"
cp -a -- "$ROOT/src" "$ROOT/scripts" "$TARGET/"
if [[ -d "$ROOT/config" ]]; then cp -a -- "$ROOT/config" "$TARGET/"; fi
find "$TARGET/src" -type f -name '*.py' -exec chmod u+x {} +
set +u
source /opt/ros/humble/setup.bash
set -u
cd -- "$TARGET"
# Rebuild on the recipient's machine; never copy the developer's build/install.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon build --executor sequential --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
set +u
source "$TARGET/install/setup.bash"
set -u
python3 "$TARGET/scripts/verify_install.py" --launch --workspace "$TARGET" --output "$TARGET/install_verification.json"
python3 - "$TARGET" "$NO_SHELL" <<'PY'
from pathlib import Path
import sys,shlex
target=Path(sys.argv[1]); q=shlex.quote(str(target))
env='source /opt/ros/humble/setup.bash\nsource '+q+'/install/setup.bash\nexport ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-90}\nexport GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://127.0.0.1:11990}\nexport LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}\nexport GAZEBO_MODEL_DATABASE_URI=""\n'
(target/'env.sh').write_text(env)
(target/'start.sh').write_text('#!/usr/bin/env bash\nset -eo pipefail\n'+env+'exec ros2 launch coal_platform_bridge coal_face_interfaces.launch.py "$@"\n')
(target/'start.sh').chmod(0o755)
if sys.argv[2]!='true':
    rc=Path.home()/'.bashrc'; old=rc.read_text() if rc.exists() else ''
    start='# >>> coal_lidar_delivery >>>'; end='# <<< coal_lidar_delivery <<<'
    if (start in old)!=(end in old):raise RuntimeError('Incomplete delivery block in .bashrc')
    block=start+'\ncoal_lidar_env() { source '+q+'/env.sh; }\ncoal-lidar() { bash '+q+'/start.sh "$@"; }\n'+end
    if start in old:
        a=old.index(start); b=old.index(end,a)+len(end); new=old[:a]+block+old[b:]
    else:new=old.rstrip()+'\n\n'+block+'\n'
    if new!=old:
        backup=target/'bashrc_before_install'
        if not backup.exists():backup.write_text(old)
        rc.write_text(new)
PY
printf '\nINSTALL PASSED: %s\n' "$TARGET"
printf 'Open a new Bash terminal: coal-lidar\n'
printf 'Sensor terminal: coal_lidar_env\n'
