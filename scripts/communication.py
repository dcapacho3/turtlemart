#!/usr/bin/env python3

import os
import subprocess
import signal
import sys
import time
from ament_index_python.packages import get_package_share_directory

# Global variable to store the tmux session name
TMUX_SESSION = "turtle_session"

def create_tmux_session():
    subprocess.run(["tmux", "new-session", "-d", "-s", TMUX_SESSION])

def run_command_in_window(command, window_name):
    subprocess.run(["tmux", "new-window", "-t", TMUX_SESSION, "-n", window_name, command])

def signal_handler(sig, frame, scripts_path, ip_address):
    print("Signal received:", sig)
    print("Terminating all processes...")

    # Run the autocloselidar_command before exit
    print("Closing lidar before exit...")
    run_command_in_window(f"python3 {os.path.join(scripts_path, 'autossh_closelidar.py')} {ip_address}", "autocloselidar")

    # Wait for 2 seconds before fully terminating
    time.sleep(10)

    # Kill the tmux session
    subprocess.run(["tmux", "kill-session", "-t", TMUX_SESSION])

    # Exit the script
    sys.exit(0)

def main():
    # Set up the signal handler
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, scripts_path, ip_address))
    signal.signal(signal.SIGTERM, lambda sig, frame: signal_handler(sig, frame, scripts_path, ip_address))

    # Define the single IP address here
    #ip_address = "192.168.8.100"  # Change this IP address as needed
    ip_address = "192.168.246.2"
    #ip_address = "10.42.0.1"  # Change this IP address as needed

    # Get the package directory
    pkg_dir = get_package_share_directory('turtlemart')
    scripts_path = os.path.join(pkg_dir, 'scripts')

    # Commands for different scripts
   
    docker_command = f"python3 {os.path.join(scripts_path, 'dockerstarter.py')}"
    bridge_command = f"python3 {os.path.join(scripts_path, 'bridgestarter.py')}"
    autossh_command = f"python3 {os.path.join(scripts_path, 'autossh.py')} {ip_address}"
    autosshloadcell_command = f"python3 {os.path.join(scripts_path, 'autossh_loadcell.py')} {ip_address}"
    autolidar_command = f"python3 {os.path.join(scripts_path, 'autossh_lidar.py')} {ip_address}"
    autocloselidar_command = f"python3 {os.path.join(scripts_path, 'autossh_closelidar.py')} {ip_address}"

    # Create a new tmux session
    create_tmux_session()


    run_command_in_window(autolidar_command, "lidar")
    time.sleep(1)
    run_command_in_window(docker_command, "docker")
    time.sleep(1)
    run_command_in_window(bridge_command, "bridge")
    time.sleep(1)
    run_command_in_window(autossh_command, "autossh")
    time.sleep(1)
    run_command_in_window(autosshloadcell_command, "loadcell")

    print(f"All processes started in tmux session '{TMUX_SESSION}'. Press Ctrl+C to terminate all.")
    print(f"To attach to the tmux session, run: tmux attach-session -t {TMUX_SESSION}")

    # Keep the main process running
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)

if __name__ == "__main__":
    main()
