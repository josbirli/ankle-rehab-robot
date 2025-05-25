# pi_bt_server.py
import bluetooth
import threading
import time
import json
from queue import Queue, Empty

from worker_threads_modified import SensorThread, ControlThread
import hardware_interface as hw
import config as config

# Queues for Inter-Thread Communication
sensor_data_to_bt_queue = Queue()
control_output_to_bt_queue = Queue() # Logs, status, errors from ControlThread
angle_update_for_control_queue = Queue() # Sensor angles for ControlThread

def sensor_angle_callback(pd_angle, ie_angle):
    sensor_data_to_bt_queue.put({"type": "angles", "pd": pd_angle, "ie": ie_angle, "ts": time.time()})
    angle_update_for_control_queue.put((pd_angle, ie_angle))

def sensor_error_callback(error_msg):
    control_output_to_bt_queue.put({"type": "error", "source": "sensor", "message": error_msg, "ts": time.time()})

def process_command_from_client(command_str, ctrl_thread):
    try:
        print(f"BT CMD RX: {command_str.strip()}")
        cmd_data = json.loads(command_str)
        cmd = cmd_data.get("command")
        params = cmd_data.get("params", {})

        if cmd == "SET_MODE":
            mode_name = params.get("mode", "MODE_IDLE") # Expect "MODE_IDLE", "MODE_MANUAL" etc
            actual_mode = getattr(config, mode_name, config.MODE_IDLE)
            ctrl_thread.set_mode(actual_mode)
        elif cmd == "SET_TARGET_ANGLES":
            ctrl_thread.set_target_angles(params.get("pd", 0.0), params.get("ie", 0.0))
        elif cmd == "START_CONTROL": ctrl_thread.start_control()
        elif cmd == "PAUSE_CONTROL": ctrl_thread.pause_control()
        elif cmd == "EMERGENCY_STOP": ctrl_thread.emergency_stop()
        elif cmd == "RESET_ESTOP": ctrl_thread.reset_emergency_stop()
        elif cmd == "SET_MANUAL_SPEED":
            ctrl_thread.set_manual_speed(params.get("speed", 50.0))
        elif cmd == "SET_ROM_PARAMS":
             ctrl_thread.set_rom_params(
                params.get("min_pd"), params.get("max_pd"),
                params.get("min_ie"), params.get("max_ie"),
                params.get("speed"), int(params.get("reps")) # Reps should be int
            )
        # Add more commands as needed
        else:
            msg = {"type": "error", "message": f"Unknown command: {cmd}", "ts": time.time()}
            control_output_to_bt_queue.put(msg)
            print(msg["message"])

    except json.JSONDecodeError:
        msg = {"type": "error", "message": "Malformed JSON command received", "ts": time.time()}
        control_output_to_bt_queue.put(msg)
        print(msg["message"])
    except Exception as e:
        msg = {"type": "error", "message": f"Error processing command '{cmd_data.get('command', 'N/A')}': {str(e)}", "ts": time.time()}
        control_output_to_bt_queue.put(msg)
        print(msg["message"])


def client_communication_thread_func(client_sock, client_info, ctrl_thread):
    print(f"Accepted connection from {client_info}")
    receive_buffer = ""
    client_active = True

    try:
        # Send initial status or welcome
        initial_status = {"type": "status", "status": ctrl_thread.current_mode, "ts": time.time()}
        client_sock.send((json.dumps(initial_status) + "\n").encode('utf-8'))

        while client_active:
            # 1. Send data from Pi to Client (non-blocking checks)
            try:
                sensor_msg = sensor_data_to_bt_queue.get_nowait()
                client_sock.send((json.dumps(sensor_msg) + "\n").encode('utf-8'))
            except Empty: pass
            except bluetooth.btcommon.BluetoothError as e: print(f"BT Send Sensor Error: {e}"); client_active = False; break

            try:
                control_msg = control_output_to_bt_queue.get_nowait()
                client_sock.send((json.dumps(control_msg) + "\n").encode('utf-8'))
            except Empty: pass
            except bluetooth.btcommon.BluetoothError as e: print(f"BT Send Control Error: {e}"); client_active = False; break

            # 2. Receive data from Client to Pi (with timeout)
            client_sock.settimeout(0.02) # Short timeout to allow sending
            try:
                data = client_sock.recv(1024)
                if not data: # Client disconnected gracefully
                    print("Client closed connection.")
                    client_active = False; break
                receive_buffer += data.decode('utf-8', errors='ignore')
                while '\n' in receive_buffer:
                    line, receive_buffer = receive_buffer.split('\n', 1)
                    if line:
                        process_command_from_client(line, ctrl_thread)
            except bluetooth.btcommon.BluetoothError as e: # Timeout is expected
                if "timed out" not in str(e).lower():
                    print(f"BT Recv Error: {e}"); client_active = False; break
            except UnicodeDecodeError as e:
                print(f"Unicode decode error on received data: {e}. Buffer: {receive_buffer}")
                receive_buffer = "" # Clear potentially corrupted buffer
            client_sock.settimeout(None) # Reset timeout


            # 3. Feed sensor angles to ControlThread
            try:
                pd, ie = angle_update_for_control_queue.get_nowait()
                ctrl_thread.update_current_angles(pd, ie)
            except Empty: pass

            if not client_active: break # Double check
            time.sleep(0.005) # Small sleep to yield

    except Exception as e:
        print(f"Error in client communication thread: {e}")
    finally:
        print(f"Closing connection with {client_info}")
        client_sock.close()


def main():
    print("Initializing Robot Hardware...")
    try:
        hw.setup_gpio()
    except Exception as e:
        print(f"FATAL: Failed to initialize hardware: {e}"); return

    print("Starting Worker Threads...")
    s_thread = SensorThread(angle_callback=sensor_angle_callback, error_callback=sensor_error_callback)
    c_thread = ControlThread(output_queue=control_output_to_bt_queue)
    s_thread.start()
    c_thread.start()

    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server_sock.bind(("", bluetooth.PORT_ANY))
    server_sock.listen(1)
    port = server_sock.getsockname()[1]
    uuid = "00001101-0000-1000-8000-00805F9B34FB" # Standard SPP UUID
    # ... inside main() before your advertise_service call
    try:
        print("Attempting minimal service advertisement for testing...")
        # Ensure server_sock is bound and listening before this
        bluetooth.advertise_service(
            server_sock,
            "TestBTService", # A simple name
            service_id = uuid, # Your existing UUID
            service_classes = [ uuid, bluetooth.SERIAL_PORT_CLASS ],
            profiles = [ bluetooth.SERIAL_PORT_PROFILE ]
            # provider = None, # Default
            # description = None, # Default
            # protocols = None # Default for SPP should be RFCOMM
        )
        print("Minimal service advertisement successful (or no immediate error).")
    except Exception as e:
        print(f"Error during minimal advertise_service test: {e}")
    # Maybe exit or handle this before proceeding to the main advertise call
# ... then your original advertise_service call

    bluetooth.advertise_service(server_sock, "AnkleRobotPi",
                               service_id=uuid,
                               service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                               profiles=[bluetooth.SERIAL_PORT_PROFILE])
    print(f"Waiting for Bluetooth connection on RFCOMM channel {port}")

    active_client_thread = None
    try:
        while True: # Keep accepting new connections if one drops
            if active_client_thread and active_client_thread.is_alive():
                time.sleep(1) # Wait if a client is being handled
                continue

            client_sock, client_info = server_sock.accept()
            active_client_thread = threading.Thread(target=client_communication_thread_func,
                                                    args=(client_sock, client_info, c_thread), daemon=True)
            active_client_thread.start()
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        if active_client_thread and active_client_thread.is_alive():
            print("Waiting for client thread to finish...")
            # You might need a more robust way to signal the client thread to stop
            active_client_thread.join(timeout=2)

        print("Stopping worker threads...")
        s_thread.stop(); c_thread.stop()
        s_thread.join(timeout=2); c_thread.join(timeout=2)
        if not s_thread.is_alive(): print("SensorThread stopped.")
        else: print("SensorThread did not stop gracefully.")
        if not c_thread.is_alive(): print("ControlThread stopped.")
        else: print("ControlThread did not stop gracefully.")

        print("Closing Bluetooth socket...")
        server_sock.close()
        print("Cleaning up GPIO...")
        hw.cleanup_gpio()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
