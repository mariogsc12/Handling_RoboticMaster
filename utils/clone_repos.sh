#!/bin/bash

# ==========================
# Default values
# ==========================
DESTINATION_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOVE_GIT=false
ORIGINAL_DIR=$(pwd)

# ==========================
# Help
# ==========================
print_help() {
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --help               Show this help message"
    echo "  --path <path>        Path to clone the repos (default: $DESTINATION_PATH)"
    echo "  --remove-git         Remove .git directories after cloning"
    echo ""
}

# ==========================
# Parse command line
# ==========================
parse_command_line() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --help)
                print_help
                exit 0
                ;;
            --path)
                DESTINATION_PATH="$2"
                shift 2
                ;;
            --remove-git)
                REMOVE_GIT=true
                shift
                ;;
            *)
                echo "Unknown option: $1"
                print_help
                exit 1
                ;;
        esac
    done
}

custom_git_clone() {
    local repo_url="$1"
    shift
    local extra_args=("$@")

    echo "Cloning repository: $repo_url"

    git clone "$repo_url" "${extra_args[@]}" || {
        echo "❌ Error cloning repository: $repo_url"
        exit 1
    }
}

clone_repos(){
    cd "$DESTINATION_PATH" || exit 1

    custom_git_clone https://github.com/elisabeth-ms/graspit_interface.git --branch noetic-devel
    custom_git_clone https://github.com/graspit-simulator/graspit_commander
    custom_git_clone https://github.com/JenniferBuehler/gazebo-pkgs --branch noetic
    custom_git_clone https://github.com/elisabeth-ms/manipulacion_pkg.git

    rm -rf gazebo-pkgs/gazebo_state_plugins
    rm -rf gazebo-pkgs/gazebo_test_tools
    rm -rf gazebo-pkgs/gazebo_world_plugin_loader
    rm -rf gazebo-pkgs/Dockerfile

    cd "$ORIGINAL_DIR" || exit 1
}

remove_git_dirs() {
    echo "Removing .git directories..."
    find "$DESTINATION_PATH" -maxdepth 6 -type d -name ".git" | while read gitdir; do
        echo "Deleting: $gitdir"
        rm -rf "$gitdir"
    done
}


main() {
    parse_command_line "$@"

    echo "Cloning repos..."
    echo "Destination path: $DESTINATION_PATH"

    clone_repos

    if [ "$REMOVE_GIT" = true ]; then
        remove_git_dirs
    fi
}

main "$@"