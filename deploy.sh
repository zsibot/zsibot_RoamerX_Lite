#!/bin/bash
# Example deployment script:
#  1. Search parent directories for a folder named zsibot_workspace.
#  2. Configure remote server information based on deployment type (navigation or steam_deck).
#  3. For each local item (directory or file) to be transferred,
#     copy it to the remote server's workspace directory while preserving the relative path starting from the local workspace.
#     If the corresponding directory does not exist on the remote server, create it first using sudo (and adjust ownership).
#  4. After deployment, for the steam_deck deployment type, use expect to invoke remote build command,
#     while for the navigation deployment type, skip the build operation.
#
# Usage:
#   ./deploy.sh <deploy_type>
#
# Parameters:
#   deploy_type: Deployment type, e.g., navigation or steam_deck

set -euo pipefail

# --- Parameter Check ---
if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <deploy_type> <remote_host>"
  echo "  navigation <remote_host>"
  echo "  steam_deck <remote_host>"
  exit 1
fi

DEPLOY_TYPE="$1"
REMOTE_HOST="$2"

# --- Configure remote server settings and local items to transfer based on deployment type ---
case "$DEPLOY_TYPE" in
navigation)
  REMOTE_USER="zsibot"
  # REMOTE_HOST set from the 2nd script argument
  REMOTE_WORKSPACE_DIR="/home/zsibot/zsibot_workspace"
  REMOTE_PASSWORD="1"
  DEFAULT_CHARACTERS="zsibot@"
  LOCAL_SUBDIRS=(
    "src/alg_common"
    "src/common"
    "src/driver"
    "src/forward"
    "src/interface"
    "src/localization"
    "src/navigation"
    "src/slam"
  )
  LOCAL_SUBDIRS_TO_SKIP=(
    # e.g., "src/navigation"
  )
  LOCAL_FILES=(
    # e.g., "README.md"
  )
  PACKAGE_LIST=(
    "alg_common"
    "common"
    "zsibot_slam"
    "laser_scan"
    "livox_driver"
    "localization"
    "navigo_behavior_tree"
    "navigo_behaviors"
    "navigo_bt_navigator"
    "navigo_collision_monitor"
    "navigo_core"
    "navigo_costmap_2d"
    "navigo_map_server"
    "navigo_mppi_controller"
    "navigo_path_controller"
    "navigo_path_planner"
    "navigo_pure_pursuit_controller"
    "navigo_spatial_temporal_planner"
    "navigo_straight_line_planner"
    "navigo_util"
    "navigo_velocity_optimizer"
    "navigo_waypoint_follower"
    "pub_tf"
    "robot_nav2"
    "robots_dog_msgs"
  )
  ;;
steam_deck)
  REMOTE_USER="zsibot"
  # REMOTE_HOST set from the 2nd script argument
  REMOTE_WORKSPACE_DIR="/home/zsibot/zsibot_workspace"
  REMOTE_PASSWORD="1"
  DEFAULT_CHARACTERS="zsibot@"
  LOCAL_SUBDIRS=(
    "src/common"
    "src/interface"
    "src/sim/src/robot_visualizer"
  )
  LOCAL_SUBDIRS_TO_SKIP=(
    "src/sim/src/robot_visualizer/robot_visualizer/map"
  )
  LOCAL_FILES=(
    # e.g., "README.md"
  )
  PACKAGE_LIST=(
    "common"
    "robot_rviz_plugins"
    "robot_visualizer"
    "robots_dog_msgs"
    "rviz_2d_overlay_msgs"
    "rviz_2d_overlay_plugins"
  )
  ;;
*)
  echo "Usage: $0 <deploy_type>"
  echo "deploy_type options: navigation, steam_deck"
  exit 1
  ;;
esac

# --- Function Definitions ---

check_requirements() {
  echo "Checking requirements …"
  local missing=0
  local cmd
  local cmds=(ssh sshpass expect)

  for cmd in "${cmds[@]}"; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
      echo "Error: '$cmd' is not installed, please retry after install '$cmd'." >&2
      missing=1
    fi
  done
  return $missing
}

# Find the zsibot_workspace directory in parent directories
check_remote_connection() {
  echo "Checking SSH connectivity to ${REMOTE_USER}@${REMOTE_HOST} …"

  local output
  local ssh_ret

  output=$(ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_USER}@${REMOTE_HOST}" exit 2>&1)
  ssh_ret=$?

  if [ $ssh_ret -ne 0 ]; then
    if echo "$output" | grep -q "WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED"; then
      echo "Host key for ${REMOTE_HOST} has changed! Removing old entry from known_hosts."
      ssh-keygen -R "${REMOTE_HOST}" >/dev/null 2>&1
    else
      echo "Warn: SSH to ${REMOTE_HOST} failed:"
      echo "  $output"
      echo "Try again …"
    fi
  fi

  /usr/bin/expect <<EOF
set timeout 15
log_user 1

spawn ssh "${REMOTE_USER}@${REMOTE_HOST}"
expect {
    -re "yes/no" {
        send "yes\r"
        exp_continue
    }
    -re "(?i)password:" {
        send "${REMOTE_PASSWORD}\r"
        exp_continue
    }
    -re "\@" {
        send "sync\r"
    }
    timeout {
        puts "Error: Timeout waiting for command execution."
        exit 1
    }
}
expect {
    -re "\@" {
        send "exit\r"
    }
    timeout {
        puts "Error: Timeout waiting for shell prompt before exit."
        exit 1
    }
}
expect {
    eof {
        catch wait result
        set exit_code [lindex \$result 3]
        if {\$exit_code != 0} {
            puts "Error: SSH command exited with status \$exit_code"
            exit \$exit_code
        }
    }
    timeout {
        puts "Error: Timeout waiting for SSH session to end."
        exit 1
    }
}
EOF

  if [ $? -ne 0 ]; then
    echo "Error: Unable to SSH into ${REMOTE_HOST}"
    return 1
  else
    echo "SSH connection to ${REMOTE_HOST} OK."
    return 0
  fi
}

find_workspace() {
  local current_dir

  current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

  WORKSPACE_DIR=""
  while [ "$current_dir" != "/" ]; do
    if [ "$(basename "$current_dir")" == "zsibot_workspace" ]; then
      WORKSPACE_DIR="$current_dir"
      break
    fi
    current_dir="$(dirname "$current_dir")"
  done

  if [ -z "$WORKSPACE_DIR" ]; then
    echo "Error: Could not find a directory named zsibot_workspace in the parent directories of the current script. Please check the directory structure."
    return 1
  else
    echo "Found local workspace: $WORKSPACE_DIR"
    return 0
  fi
}

# Construct the list of local items to transfer (combining directories and files)
construct_local_items() {
  local subdir
  local file

  LOCAL_ITEMS=()
  for subdir in "${LOCAL_SUBDIRS[@]}"; do
    LOCAL_ITEMS+=("$WORKSPACE_DIR/$subdir")
  done
  for file in "${LOCAL_FILES[@]}"; do
    LOCAL_ITEMS+=("$WORKSPACE_DIR/$file")
  done
  echo "Local items to be transferred (relative to $WORKSPACE_DIR):"
  for item in "${LOCAL_ITEMS[@]}"; do
    echo "  $item"
  done

  LOCAL_ITEMS_TO_SKIP=()
  for subdir in "${LOCAL_SUBDIRS_TO_SKIP[@]}"; do
    LOCAL_ITEMS_TO_SKIP+=("$WORKSPACE_DIR/$subdir")
  done
  echo "Local items will be skipped (relative to $WORKSPACE_DIR):"
  for item in "${LOCAL_ITEMS_TO_SKIP[@]}"; do
    echo "  $item"
  done
}

# Temp remove local items
remove_local_items() {
  local total_items
  local counter

  total_items=${#LOCAL_ITEMS_TO_SKIP[@]}
  counter=1
  for item in "${LOCAL_ITEMS_TO_SKIP[@]}"; do
    echo "--------------------------------------------"
    echo "Progress: Processing item $counter / $total_items: $item"
    rel_path="${item#"$WORKSPACE_DIR"/}"
    echo "Relative path: $rel_path"

    if [ -d "$item" ]; then
      echo "Remove files in directory $item ..."
      rm -f "$item"/*
    elif [ -f "$item" ]; then
      echo "Remove file $item ..."
      rm -f "$item"
    else
      echo "Warning: $item is neither a directory nor a file. Skipping transfer."
    fi

    if [ $? -eq 0 ]; then
      echo "Item $item removed successfully."
    else
      echo "Item $item removed failed. Please check the error message."
    fi
    counter=$((counter + 1))
  done
  echo "--------------------------------------------"
  echo "Remove completed."
}

# Transfer all local items to the remote server
transfer_items() {
  local total_items
  local counter
  local remote_target_parent

  total_items=${#LOCAL_ITEMS[@]}
  counter=1
  for item in "${LOCAL_ITEMS[@]}"; do
    echo "--------------------------------------------"
    echo "Progress: Processing item $counter / $total_items: $item"
    rel_path="${item#"$WORKSPACE_DIR"/}"
    remote_target_parent="$REMOTE_WORKSPACE_DIR/$(dirname "$rel_path")"
    if [ "$(dirname "$rel_path")" = "." ]; then
      remote_target_parent="$REMOTE_WORKSPACE_DIR"
    fi
    echo "Relative path: $rel_path"
    echo "Remote target parent directory: $remote_target_parent"
    sshpass -p "$REMOTE_PASSWORD" ssh -tt "$REMOTE_USER@$REMOTE_HOST" \
      "echo '$REMOTE_PASSWORD' | sudo -S mkdir -p '$remote_target_parent' && echo '$REMOTE_PASSWORD' | sudo -S chown -R $REMOTE_USER:$REMOTE_USER '$remote_target_parent'" </dev/null
    if [ $? -ne 0 ]; then
      echo "Failed to create or modify remote directory $remote_target_parent, skipping $item."
      counter=$((counter + 1))
      continue
    fi

    if [ -d "$item" ]; then
      echo "Transferring directory $item to remote $remote_target_parent ..."
      sshpass -p "$REMOTE_PASSWORD" scp -r "$item" "$REMOTE_USER@$REMOTE_HOST:$remote_target_parent"
    elif [ -f "$item" ]; then
      echo "Transferring file $item to remote $remote_target_parent ..."
      sshpass -p "$REMOTE_PASSWORD" scp "$item" "$REMOTE_USER@$REMOTE_HOST:$remote_target_parent"
    else
      echo "Warning: $item is neither a directory nor a file. Skipping transfer."
    fi

    if [ $? -eq 0 ]; then
      echo "Item $item transferred successfully."
    else
      echo "Item $item transfer failed. Please check the error message."
    fi
    counter=$((counter + 1))
  done
  echo "--------------------------------------------"
  echo "Transfer completed."
}

# Change the ownership of the entire remote workspace
remote_chown() {
  echo "--------------------------------------------"
  echo "Changing ownership"
  sshpass -p "$REMOTE_PASSWORD" ssh -tt "$REMOTE_USER@$REMOTE_HOST" \
    "echo '$REMOTE_PASSWORD' | sudo -S chown -R $REMOTE_USER:$REMOTE_USER '$REMOTE_WORKSPACE_DIR'" </dev/null
}

# Use expect to automatically handle remote compilation (for steam_deck deployment type)
remote_compile() {
  echo "--------------------------------------------"
  echo "Starting remote compilation..."

  local clear_build_command
  local source_command
  local build_command
  local package
  local package_string

  clear_build_command="cd '$REMOTE_WORKSPACE_DIR' && "
  for package in "${PACKAGE_LIST[@]}"; do
    clear_build_command+="rm -rf build/$package install/$package && "
  done
  clear_build_command=${clear_build_command%&& }

  source_command="cd '$REMOTE_WORKSPACE_DIR' && source install/setup.bash"

  package_string=$(
    IFS=" "
    echo "${PACKAGE_LIST[*]}"
  )
  build_command="cd '$REMOTE_WORKSPACE_DIR' && colcon build --packages-select $package_string --parallel-workers 6"

  /usr/bin/expect <<EOF
set timeout -1
log_user 1
spawn ssh ${REMOTE_USER}@${REMOTE_HOST}
expect {
    -re "(?i)(password:|\[sudo\].*的密码:)" {
        send "${REMOTE_PASSWORD}\r"
    }
    timeout {
        puts "Error: Timeout waiting for SSH password prompt."
        exit 1
    }
}
expect {
    -re "${DEFAULT_CHARACTERS}" {
        send "${clear_build_command}\r"
    }
    timeout {
        puts "Error: Timeout waiting for SSH password prompt."
        exit 1
    }
}
expect {
    -re "${DEFAULT_CHARACTERS}" {
        send "${source_command}\r"
    }
    timeout {
        puts "Error: Timeout waiting for SSH password prompt."
        exit 1
    }
}
expect {
    -re "${DEFAULT_CHARACTERS}" {
        send "${build_command}\r"
    }
    timeout {
        puts "Error: Timeout waiting for SSH password prompt."
        exit 1
    }
}
expect {
    -re "${DEFAULT_CHARACTERS}" {
        send "exit\r"
    }
    timeout {
        puts "Error: Timeout waiting for shell prompt before exit."
        exit 1
    }
}
expect {
    eof {
        catch wait result
        set exit_code [lindex \$result 3]
        if {\$exit_code != 0} {
            puts "Error: SSH command exited with status \$exit_code"
            exit \$exit_code
        }
    }
    timeout {
        puts "Error: Timeout waiting for SSH session to end."
        exit 1
    }
}
EOF
}

# --- Main Process ---
echo "Deployment type: $DEPLOY_TYPE"
echo "Remote server: $REMOTE_USER@$REMOTE_HOST"
echo "Remote workspace directory: $REMOTE_WORKSPACE_DIR"

check_requirements || {
  exit 1
}

check_remote_connection || {
  exit 1
}

find_workspace || {
  exit 1
}

construct_local_items
remove_local_items

transfer_items
remote_chown
remote_compile

echo "--------------------------------------------"
echo "Deployment completed."

exit 0
