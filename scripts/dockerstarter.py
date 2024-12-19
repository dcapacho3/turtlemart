#!/usr/bin/env python3

import subprocess
import os
import signal
import sys

# Define the Docker container name
DOCKER_CONTAINER = "myagvsuper"

# Global variable to hold the roscore process
roscore_process = None

def signal_handler(sig, frame):
    """Signal handler to terminate the roscore process and close the terminal."""
    if roscore_process:
        print("Terminating roscore process...")
        roscore_process.terminate()  # Terminate the roscore process
        roscore_process.wait()  # Wait for it to finish
    print("Closing the terminal...")
    os.system('exit')  # Close the terminal
    sys.exit(0)

def start_docker_container(container_name):
    try:
        # Attempt to start the Docker container
        subprocess.run(["docker", "start", container_name], stderr=subprocess.DEVNULL, check=True)
        print(f"Docker container '{container_name}' started.")
    except subprocess.CalledProcessError:
        print(f"Failed to start Docker container '{container_name}'.")

def run_roscore_in_docker(container_name):
    global roscore_process
    # Command to run roscore inside the Docker container
    command = f"docker exec -it {container_name} bash -c 'source /opt/ros/noetic/setup.bash; pkill roscore; roscore'"

    # Run the command directly in the current terminal
    roscore_process = subprocess.Popen(command, shell=True)
    print(f"Started roscore inside Docker container '{container_name}'.")

def main():
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)  # Handle Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Handle kill

    # Start the Docker container
    start_docker_container(DOCKER_CONTAINER)

    # Run roscore inside the Docker container in the same terminal
    run_roscore_in_docker(DOCKER_CONTAINER)

    # Keep the script running
    try:
        while True:
            pass  # Keep the script alive
    except KeyboardInterrupt:
        pass  # Allow graceful exit

if __name__ == "__main__":
    main()

