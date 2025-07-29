import time
from pymavlink import mavutil
from sensors import front, left, right, top, bottom


TERRAIN_MIN_ALTITUDE = 180  
TERRAIN_MAX_ALTITUDE = 260  
OBS = 200 
SAFE_DISTANCE = 250  
OBS_FRONT = 300
ALT = 0.7 
ALIT = 0.4
TIME = 0.05
SLEEP = 1.5
FORWARD_VELOCITY = 0.7




def connect_vehicle(connection_string='/dev/ttyACM0', baud_rate=57600):
    master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
    print("Waiting for heartbeat...")A
    master.wait_heartbeat()
    print("Vehicle connected")
    return master

def send_body_ned_velocity(master, velocity_x, velocity_y, velocity_z, duration=0.0):
    msg = master.mav.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
        1479, 0, 0, 0,
        velocity_x, velocity_y, velocity_z, 0, 0, 0, 0, 0)
    master.mav.send(msg)
    time.sleep(duration)


def set_mode(master, mode):
    mode_mapping = {"GUIDED": 4, "POSHOLD": 16}  
    if mode in mode_mapping:
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping[mode]
        )
        print(f"Setting mode to {mode}")


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

def get_ch7(master):
    msg = master.recv_match(type='RC_CHANNELS', blocking=True, timeout=1)
    return msg.chan7_raw if msg else None
def adjust_altitude_and_avoid_obstacles(master):
    front_distance = front.get_lidar_distance()
    top_distance = top.get_lidar_distance()
    left_distance = left.get_lidar_distance()
    right_distance = right_distance if right_distance is not None else float('inf')
    bottom_distance = bottom_distance if bottom_distance is not None else 230


    if front_distance >= OBS_FRONT and left_distance >= OBS and right_distance >= OBS:
        print(f"checking altitude front left right sensors distances  greater than 170")

        if bottom_distance < TERRAIN_MIN_ALTITUDE:
            print(f"bottom altitude LESS than 180: {bottom_distance} cm")
            send_body_ned_velocity(master, 0, 0, -ALIT, TIME) 
            time.sleep(SLEEP)
            print(" Increase altitude 0.5")
            send_body_ned_velocity(master, 0, 0, 0, TIME)
        elif bottom_distance > TERRAIN_MAX_ALTITUDE:
            print(f"bottom altitude greater than 250: {bottom_distance} cm")
            send_body_ned_velocity(master, 0, 0, ALIT, )  
            time.sleep(SLEEP)
            print("Decrease altitude 0.5")
            send_body_ned_velocity(master, 0, 0, 0, TIME)
        else:
            send_body_ned_velocity(master, ALT, 0, 0, TIME)  
            print("altitude is good Moving Forward")
        return
    if front_distance < OBS_FRONT:
        print(f"front obstacle detected LESS than 170: {front_distance} cm")
        if top_distance > SAFE_DISTANCE:
            send_body_ned_velocity(master, 0, 0, -ALT, TIME) 
            time.sleep(SLEEP)
            print(f"front obstacle :top sensors greater than 200: {top_distance} cm")
            send_body_ned_velocity(master, 0, 0, 0, TIME)
            print("front obs.. Move Up")
        elif left_distance > SAFE_DISTANCE:
            print(f"front obstacle :left sensor greater than 200: {left_distance} cm")

            send_body_ned_velocity(master, 0, -ALT, 0, TIME) 
            time.sleep(SLEEP)
            print("front obs.. Move left")
            send_body_ned_velocity(master, 0, 0, 0, TIME)

        elif right_distance > SAFE_DISTANCE:
            print(f"front obs right greater than 200: {right_distance} cm")
            send_body_ned_velocity(master, 0, ALT, 0, TIME) 
            time.sleep(SLEEP)
            print("front obs.. Move Right")
            send_body_ned_velocity(master, 0, 0, 0, TIME)
        return
    if left_distance < OBS:
        print(f"left obs LESS than 170: {left_distance} cm")
        if top_distance > SAFE_DISTANCE:
            print(f"lef obs :top greater than 200: {top_distance} cm")
            send_body_ned_velocity(master, 0, 0, -ALT, TIME) 
            time.sleep(SLEEP)
            print("left obs.. Move UP")
            send_body_ned_velocity(master, 0, 0, 0, TIME)
        elif right_distance > SAFE_DISTANCE:
            print(f"left obs :right greater than 200: {right_distance} cm")
            send_body_ned_velocity(master, 0, ALT, 0, TIME) 
            time.sleep(SLEEP)
            print("left obs.. Move Right")
            send_body_ned_velocity(master, 0, 0, 0, TIME)
        return

    if right_distance < OBS:
        print(f"right obs LESS than 170: {right_distance} cm")
        if top_distance > SAFE_DISTANCE:
            print(f"right obs :top greater than 200: {top_distance} cm")
            send_body_ned_velocity(master, 0, 0, -ALT, TIME)  
            time.sleep(1.5)
            print("right obs.. Move UP")
            send_body_ned_velocity(master, 0, 0, 0, TIME)
        elif left_distance > SAFE_DISTANCE:
            print(f"right obs :left greater than 200: {top_distance} cm")
            send_body_ned_velocity(master, 0, -ALT, 0, TIME)  
            time.sleep(1.5)
            print("right obs.. Move left")
            send_body_ned_velocity(master, 0, 0, 0, TIME)
        return

  
    print("no obstacles  Moving forward")
    send_body_ned_velocity(master,  FORWARD_VELOCITY, 0, 0, 0.1)
if __name__ == "__main__":
    master = connect_vehicle()

    time.sleep(1)

   
    set_mode(master, "GUIDED")
    time.sleep(1)

    
    arm_drone(master)
    time.sleep(1)

   
    takeoff(master, 2)

    try:
        while True:
            ch7_value = get_ch7(master)
            if ch7_value is None:
                print("No RC channel data received.")
                time.sleep(0.1)
                continue
            heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
            current_mode = heartbeat.custom_mode if heartbeat else None
            print(f"Current Flight Mode: {current_mode}, Ch7 Value: {ch7_value}")
            time.sleep(0.1)
            if ch7_value > 1500:
                print("Started loop > 1500")
                if current_mode not in (4, 0):  
                    set_mode(master, "GUIDED")
                    print("Switched to GUIDED mode.")
                    time.sleep(4)
               

                print("Function obs and alt started")
                adjust_altitude_and_avoid_obstacles(master)
            
            else:
                if current_mode not in (16, 0): 
                    set_mode(master, "POSHOLD")
                    time.sleep(4)
                    print("Switching to POSHOLD mode. Manual control active")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Exiting program.")                        
            
            
            
