import pexpect
import sys

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
        print("Usage: python3 autossh_closelidar.py <ip_address>")
        sys.exit(1)

    # Use command-line argument for hostname
    hostname = sys.argv[1]
    username = "er"
    password = "Elephant"
    command = "rosrun myagv_odometry close_ydlidar.sh"
    
    ssh_connect_and_execute(hostname, username, password, command)
