update_ros_workspace() {
    local workspace=$1
    
    sed -i '/# ROS Workspace/,/####################################################################/d' ~/.bashrc
    
    echo "# ROS Workspace" >> ~/.bashrc
    echo "" >> ~/.bashrc
    echo "export ROS_WORKSPACE=$workspace" >> ~/.bashrc
    echo "if [ -n \"\$ROS_WORKSPACE\" ] && [ -f \"\$ROS_WORKSPACE/install/local_setup.bash\" ]; then" >> ~/.bashrc
    echo "    source \"\$ROS_WORKSPACE/install/local_setup.bash\"" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
    echo "####################################################################" >> ~/.bashrc
    echo "" >> ~/.bashrc

}
sws() {
    if [ -z "$1" ]; then
        echo "Usage: sws <workspace_name>"
        lws
        return 1
    fi

    local full_path
    if [[ "$1" == /* ]]; then
        full_path="$1"
    else
        full_path="$HOME/$1"
    fi

    if [ -d "$full_path/install" ]; then
        update_ros_workspace "$full_path"

        export ROS_WORKSPACE=$full_path
        source "$full_path/install/local_setup.bash"
        
        echo "Switched to workspace: $full_path"
    else
        echo "Error: Invalid workspace or missing install directory: $full_path"
        lws
    fi
}

cws() {
    if [ -z "$1" ]; then
        echo "Usage: cws <workspace_name>"
        return 1
    fi

    local full_path
    if [[ "$1" == /* ]]; then
        full_path="$1"
    else
        full_path="$HOME/$1"
    fi

    if [ ! -d "$full_path" ]; then
        echo "Creating new workspace: $full_path"
        mkdir -p "$full_path/src"
        cd "$full_path"
        colcon build --symlink-install

        update_ros_workspace "$full_path"

        export ROS_WORKSPACE=$full_path
        source "$full_path/install/local_setup.bash"

        echo "Created and switched to new workspace: $full_path"
    else
        echo "Error: Workspace $full_path already exists"
    fi
}

lws() {
    echo "Available workspaces:"
    current_ws=$(echo $ROS_WORKSPACE)
    
    for workspace in ~/*_ws ~/x3cator_robot; do
        if [ -d "$workspace/install" ]; then
            if [ "$workspace" == "$current_ws" ]; then
                echo "* $(basename $workspace) (Active)"
            else
                echo "  $(basename $workspace)"
            fi
        fi
    done
    
    if [ -z "$current_ws" ]; then
        echo "No workspace currently selected"
    fi
}

get_current_ws() {
    echo "$ROS_WORKSPACE"
}

alias cw='cd "$(get_current_ws)"'
alias cs='cd "$(get_current_ws)/src"'