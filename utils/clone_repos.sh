#!/bin/bash

# ==========================
# Default values
# ==========================
DEFAULT_DESTINATION_PATH="."

DESTINATION_PATH="$DESTINATION_PATH"

# ==========================
# Help
# ==========================
print_help() {
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --help               Show this help message"
    echo "  --path               Path to clone the repos (default: $DESTINATION_PATH)"
    echo ""
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
            *)
                echo "Unknown option: $1"
                print_help
                exit 1
                ;;
        esac
    done
}

clone_repos(){
    git clone https://github.com/elisabeth-ms/graspit_interface.git --branch noetic-devel
    git clone https://github.com/graspit-simulator/graspit_commander
    git clone https://github.com/JenniferBuehler/gazebo-pkgs --branch noetic
}

main() {
    parse_command_line "$@"

    echo "Cloning repos..."
    echo "Destination path: $DESTINATION_PATH"

    clone_repos
}

main "$@"