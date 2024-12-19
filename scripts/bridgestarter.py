#!/usr/bin/env python3

import subprocess
import os
import signal
import sys

# Global variable to hold the process
bridge_process = None

def signal_handler(sig, frame):
    """Signal handler to terminate the ros1_bridge process and close the terminal."""
    if bridge_process:
        print("Terminating ros1_bridge process...")
        bridge_process.terminate()  # Terminate the child process
        bridge_process.wait()  # Wait for it to finish
    print("Closing the terminal...")
    os.system('exit')  # Close the terminal
    sys.exit(0)

def run_ros1_bridge():
    global bridge_process
    # Run the ros1_bridge dynamic_bridge command
    #bridge_process = subprocess.Popen(["ros2", "run", "ros1_bridge", "dynamic_bridge", "--bridge-all-topics"])
    bridge_process = subprocess.Popen(["ros2", "launch", "ros1_ros2_bridge", "bridge.launch.py"])
    print("Started ros1_bridge dynamic_bridge process.")

def main():
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)  # Handle Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Handle kill

    # Execute the ros1_bridge command
    run_ros1_bridge()

    # Keep the script running
    try:
        while True:
            pass  # Keep the script alive
    except KeyboardInterrupt:
        pass  # Allow graceful exit

if __name__ == "__main__":
    main()

