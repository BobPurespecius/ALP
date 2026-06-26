#!/bin/bash

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DISABLE_ROS1_EOL_WARNINGS=1

append_path_once() {
    local var_name="$1"
    local path_value="$2"

    if [ -z "$path_value" ] || [ ! -e "$path_value" ]; then
        return
    fi

    eval "local current_value=\"\${$var_name:-}\""
    case ":$current_value:" in
        *":$path_value:"*) ;;
        *)
            if [ -z "$current_value" ]; then
                export "$var_name=$path_value"
            else
                export "$var_name=$current_value:$path_value"
            fi
            ;;
    esac
}

prepend_path_once() {
    local var_name="$1"
    local path_value="$2"

    if [ -z "$path_value" ] || [ ! -e "$path_value" ]; then
        return
    fi

    eval "local current_value=\"\${$var_name:-}\""
    local new_value=""
    local entry

    IFS=':' read -ra entries <<< "$current_value"
    for entry in "${entries[@]}"; do
        if [ -z "$entry" ] || [ "$entry" = "$path_value" ]; then
            continue
        fi
        if [ -z "$new_value" ]; then
            new_value="$entry"
        else
            new_value="$new_value:$entry"
        fi
    done

    if [ -z "$new_value" ]; then
        export "$var_name=$path_value"
    else
        export "$var_name=$path_value:$new_value"
    fi
}

source_if_exists() {
    local setup_file="$1"
    shift

    if [ -f "$setup_file" ]; then
        # shellcheck disable=SC1090
        source "$setup_file" "$@"
    fi
}

find_px4_home() {
    local candidate
    for candidate in \
        "${PX4_HOME:-}" \
        "/usr/src/PX4-Autopilot" \
        "/home/developer/PX4_Firmware" \
        "/home/developer/PX4-Autopilot" \
        "/home/bob/PX4_Firmware" \
        "/home/bob/PX4-Autopilot"
    do
        if [ -n "$candidate" ] && [ -f "$candidate/package.xml" ]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

find_ego_home() {
    local candidate
    for candidate in \
        "$workspace_dir/src/ego-planner" \
        "$workspace_dir/../../ego-planner" \
        "/app/guidance/ros_ws/src/ego-planner" \
        "/app/ego-planner" \
        "/home/developer/ego-planner" \
        "/home/bob/ego-planner"
    do
        if [ -n "$candidate" ] && [ -d "$candidate/src" ]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

source_if_exists "/opt/ros/noetic/setup.bash"
source_if_exists "$workspace_dir/devel/setup.bash"
prepend_path_once ROS_PACKAGE_PATH "$workspace_dir/src"
prepend_path_once CMAKE_PREFIX_PATH "$workspace_dir/devel"
prepend_path_once LD_LIBRARY_PATH "/usr/local/lib"

ego_home="$(find_ego_home || true)"
if [ -n "$ego_home" ]; then
    source_if_exists "$ego_home/devel/setup.bash" --extend
    prepend_path_once ROS_PACKAGE_PATH "$ego_home/src"
    prepend_path_once CMAKE_PREFIX_PATH "$ego_home/devel"
    prepend_path_once PYTHONPATH "$ego_home/devel/lib/python3/dist-packages"
    prepend_path_once LD_LIBRARY_PATH "$ego_home/devel/lib"
fi

px4_home="$(find_px4_home || true)"
if [ -n "$px4_home" ]; then
    export PX4_HOME="$px4_home"
    if [ -f "$px4_home/Tools/simulation/gazebo-classic/setup_gazebo.bash" ]; then
        # shellcheck disable=SC1090
        source "$px4_home/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
            "$px4_home" "$px4_home/build/px4_sitl_default"
        prepend_path_once ROS_PACKAGE_PATH "$px4_home/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
        append_path_once GAZEBO_MODEL_PATH "$px4_home/Tools/simulation/gazebo-classic/models"
        append_path_once GAZEBO_RESOURCE_PATH "$px4_home/Tools/simulation/gazebo-classic"
    elif [ -f "$px4_home/Tools/setup_gazebo.bash" ]; then
        # shellcheck disable=SC1090
        source "$px4_home/Tools/setup_gazebo.bash" \
            "$px4_home" "$px4_home/build/px4_sitl_default"
        prepend_path_once ROS_PACKAGE_PATH "$px4_home/Tools/sitl_gazebo"
        append_path_once GAZEBO_MODEL_PATH "$px4_home/Tools/sitl_gazebo/models"
        append_path_once GAZEBO_RESOURCE_PATH "$px4_home/Tools/sitl_gazebo"
    fi

    prepend_path_once ROS_PACKAGE_PATH "$px4_home"
    append_path_once LD_LIBRARY_PATH "$px4_home/build/px4_sitl_default/build_gazebo"
fi

append_path_once GAZEBO_PLUGIN_PATH "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
prepend_path_once LD_LIBRARY_PATH "/usr/local/lib"

# Keep the paths inferred from this checkout ahead of stale absolute paths that
# may be baked into old catkin devel spaces.
prepend_path_once ROS_PACKAGE_PATH "$workspace_dir/src"
prepend_path_once CMAKE_PREFIX_PATH "$workspace_dir/devel"

if [ -n "$ego_home" ]; then
    prepend_path_once ROS_PACKAGE_PATH "$ego_home/src"
    prepend_path_once CMAKE_PREFIX_PATH "$ego_home/devel"
    prepend_path_once PYTHONPATH "$ego_home/devel/lib/python3/dist-packages"
    prepend_path_once LD_LIBRARY_PATH "$ego_home/devel/lib"
fi

if [ -n "$px4_home" ]; then
    prepend_path_once ROS_PACKAGE_PATH "$px4_home"
    if [ -d "$px4_home/Tools/simulation/gazebo-classic/sitl_gazebo-classic" ]; then
        prepend_path_once ROS_PACKAGE_PATH "$px4_home/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    elif [ -d "$px4_home/Tools/sitl_gazebo" ]; then
        prepend_path_once ROS_PACKAGE_PATH "$px4_home/Tools/sitl_gazebo"
    fi
fi
