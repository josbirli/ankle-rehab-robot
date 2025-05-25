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
    advertised = False

    try:
        server_sock.bind(("", bluetooth.PORT_ANY))
        server_sock.listen(1)
        port = server_sock.getsockname()[1]
        uuid = "00001101-0000-1000-8000-00805F9B34FB"

        print(f"Attempting to advertise service on RFCOMM channel {port}")
        bluetooth.advertise_service(server_sock, "AnkleRobotPi",
                                   service_id=uuid,
                                   service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                                   profiles=[bluetooth.SERIAL_PORT_PROFILE])
        advertised = True
        print(f"Service 'AnkleRobotPi' advertised on RFCOMM channel {port}")
        print("Waiting for Bluetooth connection...")

        active_client_thread = None
        while True:
            if active_client_thread and active_client_thread.is_alive():
                time.sleep(0.5) # Check less frequently
                continue

            # Consider adding a timeout to accept() if you want the main loop
            # to be able to break on KeyboardInterrupt more readily while waiting
            # server_sock.settimeout(1.0) # 1 second timeout
            # try:
            # client_sock, client_info = server_sock.accept()
            # except bluetooth.btcommon.BluetoothError as e:
            # if "timed out" in str(e).lower():
            # continue # Go back to checking loop conditions
            # else:
            # raise # Re-raise other Bluetooth errors
            # server_sock.settimeout(None) # Reset timeout

            client_sock, client_info = server_sock.accept() # Blocking call
            
            # If a previous client thread finished, ensure it's joined before starting a new one
            if active_client_thread and not active_client_thread.is_alive():
                active_client_thread.join() # Clean up the finished thread resource

            active_client_thread = threading.Thread(target=client_communication_thread_func,
                                                    args=(client_sock, client_info, c_thread), daemon=True)
            active_client_thread.start()
            print(f"Client {client_info} connected, handler thread started.")


    except KeyboardInterrupt:
        print("\nShutting down server due to KeyboardInterrupt...")
    except Exception as e:
        print(f"An unexpected error occurred in main loop: {e}")
    finally:
        if advertised:
            print("Stopping service advertisement...")
            try:
                bluetooth.stop_advertising(server_sock)
                print("Service advertisement stopped.")
            except Exception as e:
                print(f"Error stopping advertisement: {e}")
        
        # Client thread cleanup logic might need improvement if KeyboardInterrupt happens
        # while a client is connected and the thread is running.
        # A more robust solution would involve signaling the client thread to stop.
        if 'active_client_thread' in locals() and active_client_thread and active_client_thread.is_alive():
            print("Attempting to clean up active client thread (may take a moment if blocked)...")
            # This join might block if the client_communication_thread_func is stuck.
            # Robustly stopping it would require an event or flag inside that thread.
            active_client_thread.join(timeout=2) 
            if active_client_thread.is_alive():
                print("Warning: Client thread did not exit cleanly.")


        print("Stopping worker threads...")
        if 's_thread' in locals() and s_thread.is_alive(): s_thread.stop()
        if 'c_thread' in locals() and c_thread.is_alive(): c_thread.stop()
        if 's_thread' in locals(): s_thread.join(timeout=2)
        if 'c_thread' in locals(): c_thread.join(timeout=2)
        # ... (print thread stop status) ...

        print("Closing Bluetooth server socket...")
        if 'server_sock' in locals(): server_sock.close()
        print("Cleaning up GPIO...")
        hw.cleanup_gpio()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
