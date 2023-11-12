import os
import sys

def create_config_file(ssid, password, server_dest, server_port):
    content = f"""\
    #ifndef CONFIG_H
    #define CONFIG_H

    // Update these values with your specific IP address and port
    #define NETWORK_SSID "{ssid}"
    #define NETWORK_PASSWORD "{password}"
    #define SERVER_DEST "{server_dest}"
    #define SERVER_PORT {server_port}

    #endif  // CONFIG_H
    """

    file_path = os.path.join(os.getcwd(), "config.h")

    with open(file_path, "w") as file:
        file.write(content)

    print(f"Config file 'config.h' created at: {file_path}")

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python create_config_file.py <ssid> <password> <server_dest> <server_port>")
    else:
        ssid, password, server_dest, server_port = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
        create_config_file(ssid, password, server_dest, server_port)
