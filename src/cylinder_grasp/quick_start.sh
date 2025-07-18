#!/bin/bash

# Quick Start Script for Cylinder Grasp Controller
# This script provides easy commands to run the cylinder grasping system

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if ROS is sourced
check_ros_setup() {
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS environment not sourced. Please run:"
        echo "source /opt/ros/<your_ros_distro>/setup.bash"
        echo "source ~/catkin_ws/devel/setup.bash"
        exit 1
    fi
    print_info "ROS $ROS_DISTRO environment detected"
}

# Function to check if the package exists
check_package() {
    if ! rospack find cylinder_grasp > /dev/null 2>&1; then
        print_error "Package 'cylinder_grasp' not found. Please make sure it's built and in your workspace."
        exit 1
    fi
    print_info "Package 'cylinder_grasp' found"
}

# Function to start the controller
start_controller() {
    print_info "Starting cylinder grasping controller..."
    rosrun cylinder_grasp ros_cylinder_grasp.py &
    CONTROLLER_PID=$!
    sleep 2
    
    if kill -0 $CONTROLLER_PID 2>/dev/null; then
        print_success "Controller started successfully (PID: $CONTROLLER_PID)"
        return 0
    else
        print_error "Failed to start controller"
        return 1
    fi
}

# Function to start the tester
start_tester() {
    print_info "Starting test interface..."
    rosrun cylinder_grasp test_cylinder_grasp.py
}

# Function to run automated tests
run_auto_test() {
    print_info "Running automated tests..."
    rosrun cylinder_grasp test_cylinder_grasp.py auto
}

# Function to start with launch file
start_with_launch() {
    print_info "Starting with launch file..."
    roslaunch cylinder_grasp cylinder_grasp.launch
}

# Function to start with debug mode
start_debug() {
    print_info "Starting in debug mode..."
    roslaunch cylinder_grasp cylinder_grasp.launch debug:=true
}

# Function to start with RViz
start_with_rviz() {
    print_info "Starting with RViz visualization..."
    roslaunch cylinder_grasp cylinder_grasp.launch use_rviz:=true
}

# Function to show help
show_help() {
    echo -e "${BLUE}Cylinder Grasp Controller - Quick Start Script${NC}"
    echo ""
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  start           Start controller and test interface"
    echo "  controller      Start only the controller"
    echo "  test            Start only the test interface"
    echo "  auto-test       Run automated tests"
    echo "  launch          Start with launch file"
    echo "  debug           Start in debug mode"
    echo "  rviz            Start with RViz visualization"
    echo "  services        Show available ROS services"
    echo "  topics          Show available ROS topics"
    echo "  kill            Kill all related processes"
    echo "  help            Show this help message"
    echo ""
    echo "Quick commands after starting:"
    echo "  rosservice call /start_grasp         # Start grasping"
    echo "  rosservice call /start_viewer        # Start MuJoCo viewer"
    echo "  rosservice call /reset_grasp         # Reset system"
    echo "  rosservice call /stop_grasp          # Stop grasping"
    echo ""
}

# Function to show services
show_services() {
    print_info "Available ROS services:"
    rosservice list | grep -E "(start_grasp|reset_grasp|stop_grasp|start_viewer|stop_viewer)" || echo "No services found. Make sure the controller is running."
}

# Function to show topics
show_topics() {
    print_info "Available ROS topics:"
    rostopic list | grep -E "(joint_states|cylinder_pose|grasp_status|debug_info|joint_commands|position_commands)" || echo "No topics found. Make sure the controller is running."
}

# Function to kill processes
kill_processes() {
    print_info "Killing cylinder grasp processes..."
    pkill -f "ros_cylinder_grasp.py"
    pkill -f "test_cylinder_grasp.py"
    print_success "Processes killed"
}

# Function to start everything
start_all() {
    print_info "Starting cylinder grasping system..."
    
    # Start controller in background
    if start_controller; then
        sleep 3
        
        # Start viewer
        print_info "Starting MuJoCo viewer..."
        rosservice call /start_viewer
        sleep 2
        
        # Start test interface
        print_info "Starting test interface..."
        print_info "You can now use the test interface to control the system"
        start_tester
    else
        print_error "Failed to start system"
        exit 1
    fi
}

# Main script logic
main() {
    # Check prerequisites
    check_ros_setup
    check_package
    
    case "$1" in
        "start")
            start_all
            ;;
        "controller")
            start_controller
            echo "Controller started. Press Ctrl+C to stop."
            wait
            ;;
        "test")
            start_tester
            ;;
        "auto-test")
            run_auto_test
            ;;
        "launch")
            start_with_launch
            ;;
        "debug")
            start_debug
            ;;
        "rviz")
            start_with_rviz
            ;;
        "services")
            show_services
            ;;
        "topics")
            show_topics
            ;;
        "kill")
            kill_processes
            ;;
        "help"|"--help"|"-h")
            show_help
            ;;
        "")
            print_warning "No option specified. Showing help:"
            show_help
            ;;
        *)
            print_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
}

# Handle Ctrl+C
trap 'print_info "Shutting down..."; kill_processes; exit 0' INT

# Run main function
main "$@"