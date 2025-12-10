#!/usr/bin/env python3
import sys
import subprocess
import os
import argparse

def configure_camera(utility_path, mac, ip, subnet, gateway, camera_name):
    """Executes the IpConfigUtility command for a single camera based on its MAC."""
    if utility_path is None or not os.path.exists(utility_path):
        print("ERROR: Utility path is not set or does not exist.")
        print("Set the ARENA_SDK_UTILS_PATH environment variable by running 'export ARENA_SDK_UTILS_PATH=...' or use --utility-path flag.")
        return 1
    # Check if MAC and IP are provided (to handle single-camera modes, where they are '')
    if not mac or not ip:
        print(f"Skipping configuration for {camera_name}: MAC or IP not provided.")
        return 0
        
    # The utility path should point to the executable file
    utility_full_path = os.path.join(utility_path, 'IpConfigUtility')
    
    if not os.path.exists(utility_full_path):
        print(f"ERROR: Utility not found at {utility_full_path}. Check ARENA_SDK_UTILITIES_PATH.")
        return 1

    # Construct the full command. We must prepend 'sudo' as network changes require root privileges.
    cmd = [
        utility_full_path,
        '/force',
        '-m', mac,
        '-a', ip,
        '-s', subnet,
        '-g', gateway
    ]

    print(f"\n--- Configuring {camera_name} ({mac}) ---")
    print(f"Command: {' '.join(cmd)}")
    
    try:
        # CRITICAL FIX: Replaced stdin=subprocess.DEVNULL with input='\n'.
        # This sends a newline character (ENTER) to the utility's input stream, 
        # satisfying the "Press enter to complete" prompt.
        result = subprocess.run(
            cmd, 
            check=True, 
            capture_output=True, 
            text=True,
            input='\n', # <--- FIX: Pipes a newline character (ENTER) to the utility
            timeout=30 # Set a max runtime
        )
        # --- VERIFICATION CHANGE ---
        print("\n--- Utility Output ---")
        print(result.stdout.strip())
        print("------------------------")
        print(f"Configuration successful for {camera_name}.")
        # --- END VERIFICATION CHANGE ---
        return 0
    except subprocess.TimeoutExpired:
        print(f"ERROR: IPConfigUtility timed out after 10 seconds for {camera_name}. Check network connection.")
        return 1
    except subprocess.CalledProcessError as e:
        print(f"ERROR: IPConfigUtility failed for {camera_name}. Return code {e.returncode}.")
        print(f"Stderr:\n{e.stderr.strip()}")
        return 1
    except Exception as e:
        print(f"FATAL ERROR during subprocess execution: {e}")
        return 1


def main():
    parser = argparse.ArgumentParser(description="Configures camera IP addresses based on provided details.")

    # Load defaults from config/network_config.yaml if available
    config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', 'network_config.yaml'))
    _cfg = {}
    if os.path.exists(config_path):
        try:
            try:
                import yaml
            except ImportError:
                yaml = None
                print(f"WARNING: PyYAML not installed. Ignoring config file {config_path}.")

            if yaml:
                with open(config_path, 'r') as cf:
                    loaded = yaml.safe_load(cf)
                    if isinstance(loaded, dict):
                        _cfg = loaded
        except Exception as e:
            print(f"WARNING: Failed to load config file {config_path}: {e}")
    
    # Define Named Arguments
    # resolve env-var name and actual path from the YAML
    utility_env_name = _cfg.get('arena_sdk_path_env', 'ARENA_SDK_UTILS_PATH')
    utility_default = os.getenv(utility_env_name)

    parser.add_argument(
        '--utility-path',
        default=utility_default,
        help=f"Base path to Arena SDK utilities (env var: {utility_env_name})."
    )

    # optional: check early with a temp parse (or do one final parse at the end)
    temp_args, _ = parser.parse_known_args()
    if temp_args.utility_path and not os.path.exists(os.path.join(temp_args.utility_path, 'IpConfigUtility')):
        print(f"WARNING: IpConfigUtility not found at {temp_args.utility_path} (override with --utility-path)")

    # nested camera defaults
    _left = _cfg.get('cameras', {}).get('triton_left', {})
    _right = _cfg.get('cameras', {}).get('triton_right', {})

    parser.add_argument('--subnet', default=_cfg.get('subnet_mask', '255.255.255.0'))
    parser.add_argument('--gateway', default=_cfg.get('gateway', '0.0.0.0'))

    parser.add_argument('--left-mac', default=_left.get('mac', ''), help="MAC address of the left camera.")
    parser.add_argument('--left-ip', default=_left.get('ip', '10.42.0.201'), help="IP for left camera.")
    parser.add_argument('--right-mac', default=_right.get('mac', ''), help="MAC address of the right camera.")
    parser.add_argument('--right-ip', default=_right.get('ip', '10.42.0.202'), help="IP for right camera.")
    
    parser.add_argument('--mode', required=True, choices=['left', 'right', 'single_cam', 'dual_cam'], help="Operation mode (left, right, single_cam or dual_cam).")
    
    args = parser.parse_args()
    
    mode = args.mode.lower()
    
    print(f"Starting camera IP configuration (Mode: {mode})...")

    # --- Configure Left Camera ---
    if mode in ["left", "single_cam", "dual_cam"]:
        if configure_camera(args.utility_path, args.left_mac, args.left_ip, args.subnet, args.gateway, "Triton Left") != 0:
            sys.exit(1)
    
    # --- Configure Right Camera ---
    if mode in ["right", "dual_cam"]:
        if configure_camera(args.utility_path, args.right_mac, args.right_ip, args.subnet, args.gateway, "Triton Right") != 0:
            sys.exit(1)
        
    print("\nIP assignment phase completed.")
    sys.exit(0)

if __name__ == '__main__':
    main()