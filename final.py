from pymavlink import mavutil

def send_guided_commands(vehicle, latitude, longitude):
    #current_altitude = get_current_altitude(vehicle)  # Get the current altitude
    msg = vehicle.mav.set_position_target_global_int_encode(
        0,           # time_boot_ms (not used)
        0,           # target_system (not used)
        0,           # target_component (not used)
        6,  # frame
        0b110111111000  ,  # type_mask (only positions)  0b110111111000
        int(latitude ),  # lat_int
        int(longitude ),  # lon_int
        3,  # Use the current altitude as the target altitude (in millimeters)
        0,           # vx (not used)
        0,           # vy (not used)
        0,           # vz (not used)
        0,           # afx (not used)
        0,           # afy (not used)
        0,           # afz (not used)
        0,           # yaw (not used)
        0            # yaw_rate (not used)
    )

    vehicle.mav.send(msg)
    #vehicle.flush()

# Function to read MAVLink GPS messages
def read_gps_data(connection_telemetry,connection_flight_controller):
    while True:
        msg = connection_telemetry.recv_match()

        if msg:
            if msg.get_type() == 'GPS_RAW_INT':
                latitude = msg.lat 
                longitude = msg.lon 
                #print(f'Latitude: {latitude}, Longitude: {longitude}')
                monitor_flight_mode(connection_flight_controller,latitude,longitude)

# Function to monitor flight mode
def monitor_flight_mode(connection,lat,lon):
    msg = connection.recv_match(type='HEARTBEAT', blocking=True)

    if msg:
        flight_mode = mavutil.mode_string_v10(msg)
        print(flight_mode)
        print(int(lat),' ',int(lon))
        if flight_mode=='GUIDED':
            send_guided_commands(connection, lat, lon)
                    
            

# Create a MAVLink connection for telemetry
connection_telemetry = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
connection_telemetry.wait_heartbeat()
print("Telemetry Heartbeat from system (system %u component %u)" % (connection_telemetry.target_system, connection_telemetry.target_component))

# Create a MAVLink connection for flight controller
connection_flight_controller = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
connection_flight_controller.wait_heartbeat()
print("Flight Controller Heartbeat from system (system %u component %u)" % (connection_flight_controller.target_system, connection_flight_controller.target_component))

# Run the code snippets
read_gps_data(connection_telemetry,connection_flight_controller)
#monitor_flight_mode(connection_flight_controller)
