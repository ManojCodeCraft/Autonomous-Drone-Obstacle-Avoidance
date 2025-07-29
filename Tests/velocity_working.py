import time
from pymavlink import mavutil

def connect_vehicle():
    print("Connecting to vehicle...")
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600)
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print("Vehicle connected")
    return master

def set_mode(master, mode):
    mode_id = master.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Mode {mode} not available")
        return
    master.set_mode(mode_id)
    print(f"Mode set to {mode}")

def arm_drone(master):
    print("Arming drone...")
    master.arducopter_arm()
    
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone armed!")
            break
        time.sleep(0.5)

def takeoff(master, altitude):
    print(f"Taking off to {altitude} meters...")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1, 1
        )

        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            current_altitude = msg.relative_alt / 1000.0  # Convert mm to meters
            print(f"Current Altitude: {current_altitude:.2f} meters")
            if current_altitude >= altitude * 0.95:
                print("Reached target altitude!")
                break
        else:
            print("No altitude data received. Retrying...")
        
        time.sleep(0.5)
def send_body_ned_velocity(master, velocity_x, velocity_y, velocity_z, duration=0.05):
    msg = master.mav.set_position_target_local_ned_encode(
        0,       
        0, 0, 
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        1479, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 
        0, 0)
    master.mav.send(msg)
    time.sleep(duration)

def land_drone(master):
    print("Initiating landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Landing in progress...")
    time.sleep(10)
    print("Landed successfully!")

if __name__ == "__main__":
    master = connect_vehicle()
    time.sleep(1)

    set_mode(master, "GUIDED")
    time.sleep(1)
    
    arm_drone(master)
    time.sleep(1)

    
    takeoff(master, 2)

    print("Takeoff complete. Moving...")

    
    send_body_ned_velocity(master, velocity_x=1, velocity_y=0, velocity_z=0, duration=0.05)
    time.sleep(1.5)

    send_body_ned_velocity(master, velocity_x=-1, velocity_y=0, velocity_z=0, duration=0.05)
    time.sleep(1.5)

    
    send_body_ned_velocity(master, velocity_x=0, velocity_y=1, velocity_z=0, duration=0.05)
    time.sleep(1.5)

    
    send_body_ned_velocity(master, velocity_x=0, velocity_y=-1, velocity_z=0, duration=0.05)
    time.sleep(1.5)

    send_body_ned_velocity(master, velocity_x=0, velocity_y=0, velocity_z=0, duration=0.05)
    time.sleep(1.5)
    
    print("Movement complete. Landing...")

    
    land_drone(master)
    
    master.close()  
