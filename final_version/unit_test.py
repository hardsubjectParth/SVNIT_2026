from pymavlink import mavutil
import mavsdk as sdk 
import asyncio 
# global variable
SET_TAKEOF_ALTITUDE = 3.0
statement1 = "Arming check...." 

'''
#! regular functions

'''

async def setting_current_postion_as_home():
    ## printing teh current cordinates 
    print("Current coordinates are : ")
    drone = sdk.System()
    await drone.connect(system_address='udp://:14540')
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break 
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("GPS is looking good")
            break

    print("Reading GPS position...")
    async for position in drone.telemetry.position():
        print(f"Latitude:  {position.latitude_deg}")
        print(f"Longitude: {position.longitude_deg}")
        print(f"Altitude (AMSL): {position.absolute_altitude_m} m")
        print(f"Altitude (Relative): {position.relative_altitude_m} m")
        break # remove this to get real time coordinate data .. 

        # to update this block of code .. 
    

async def arm_check():
    drone = sdk.System()
    await drone.connect(system_address='udp://:14540')
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

# arms and disarm check
    while True:
        print("starting arming check...")
        await drone.action.arm() 
        await asyncio.sleep(2)
        await drone.action.disarm()

    print("arming check done ")

# Take off and land check
async def TakeOfLandCheck():
    drone = sdk.System()
    await drone.connect(system_address='udp://:14540')
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break
    
    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position estimate OK")
            break

    # Take of logic 
    '''seting a takeoff altitude , to which drone will go and takeoff '''
    await drone.action.set_takeoff_altitude(SET_TAKEOF_ALTITUDE)
    print("Arming drone")
    drone.action.arm()
    await asyncio.sleep(1)
    # Take of to the set_takeoff_altitude
    print("Taking off")
    await asyncio.sleep(8) # wait till the drone is taking off and hovering at the same altitude 
    print("Landing...")
    # Landing the drone
    await drone.action.land()
    await asyncio.sleep(10) # if landed then this will get exectued and drone will go to sleep mode ..
    print("Take off hover Test is complete")


    



    
    
    



