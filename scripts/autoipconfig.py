import pexpect
import sys
import subprocess

def get_primary_ip():
    """Get the primary IP address using the hostname command."""
    try:
        # Run the command and capture the output
        result = subprocess.run(["hostname", "-I"], capture_output=True, text=True)
        ip_addresses = result.stdout.strip().split()

        if ip_addresses:
            # Return the first IP address (usually the main one)
            return ip_addresses[0]
        else:
            return "127.0.0.1"  # Fallback to localhost
    except Exception as e:
        print(f"Failed to get IP address: {str(e)}")
        return "127.0.0.1"  # Fallback to localhost if there's an error


def ssh_connect_and_execute(hostname, username, password, initial_command):
    # Construct the SSH command
    ssh_command = f'ssh {username}@{hostname}'

    try:
        # Spawn the SSH process
        child = pexpect.spawn(ssh_command, encoding='utf-8')
        child.logfile = sys.stdout  # This will print all output to stdout

        # Handle the password prompt
        i = child.expect(['password:', 'continue connecting (yes/no)?'], timeout=30)
        if i == 1:
            child.sendline('yes')
            child.expect('password:')
        child.sendline(password)

        # Wait for the shell prompt
        child.expect('[$#] ')

        # Send the initial command
        child.sendline(initial_command)

        # Enter interactive mode
        while True:
            i = child.expect([pexpect.TIMEOUT, pexpect.EOF, '[$#] '], timeout=1)
            if i == 0:  # Timeout
                print(child.before, end='', flush=True)
            elif i == 1:  # EOF
                print("SSH connection closed.")
                break
            elif i == 2:  # Command completed
                print(child.before, end='', flush=True)
                user_input = input()
                if user_input.lower() in ['exit', 'quit', 'bye']:
                    child.sendline('exit')
                    print("Exiting SSH session.")
                    break
                child.sendline(user_input)

    except pexpect.exceptions.TIMEOUT:
        print("Error: SSH connection timed out")
    except Exception as e:
        print(f"An error occurred: {str(e)}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 autoipconfig.py <ip_address>")
        sys.exit(1)

    # Use command-line argument for hostname
    hostname = sys.argv[1]
    username = "er"
    password = "Elephant"
 # Get the primary IP address of the local machine
    local_ip = get_primary_ip()

    # Set the command to change ROS_MASTER_URI to the primary local IP address
    command = f"export ROS_MASTER_URI=http://{local_ip}:11311/"

    print(f"Setting ROS_MASTER_URI to: {command}")

    
    ssh_connect_and_execute(hostname, username, password, command)
