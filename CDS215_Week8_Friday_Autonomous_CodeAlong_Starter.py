"""
CDS215 Week 8 Friday: Autonomous Husky Multi-Destination Challenge

The Husky autonomously navigates to multiple destinations in order.

Requirements:
    pip install pybullet

Run:
    python CDS215_Husky_Multiple_Destinations.py
"""

import math
import time
import pybullet as p
import pybullet_data


# ============================================================
# STEP 1: Define navigation settings
# ============================================================

GOAL_RADIUS =  .65
SPEED = 5
TURN_SPEED = 4
HEADING_TOLERANCE = .14

LEFT_WHEELS = [2, 4]
RIGHT_WHEELS = [3, 5]


# ============================================================
# NEW STEP 1A: Create a list of destinations
# ============================================================

#Create list of destinations
waypoints = [(4,2),(6,7),(2,8)]

# Keep track of which destination we are visiting.
current_target = 0

# Get the coordinates of our first destination.
TARGET_X, TARGET_Y = waypoints[current_target]

# ============================================================
# Robot movement functions (unchanged)
# ============================================================

def set_wheel_speeds(robot_id, left, right):
    for joint in LEFT_WHEELS:
        p.setJointMotorControl2(
            robot_id,
            joint,
            p.VELOCITY_CONTROL,
            targetVelocity=left,
            force=100
        )

    for joint in RIGHT_WHEELS:
        p.setJointMotorControl2(
            robot_id,
            joint,
            p.VELOCITY_CONTROL,
            targetVelocity=right,
            force=100
        )


def stop_robot(robot_id):
    set_wheel_speeds(robot_id, 0, 0)


def move_forward(robot_id):
    set_wheel_speeds(robot_id, SPEED, SPEED)


def turn_left(robot_id):
    set_wheel_speeds(robot_id, -TURN_SPEED, TURN_SPEED)


def turn_right(robot_id):
    set_wheel_speeds(robot_id, TURN_SPEED, -TURN_SPEED)

##
##
#finish defining function below
def angle_difference(target_angle, current_angle):
   return math.atan2(
       math.sin(target_angle - current_angle),
        math.cos(target_angle - current_angle) )
    

# ============================================================
# NEW STEP 2: Display all destinations
# ============================================================

def add_target_markers():

#i is index of waypoint, x coordinate, y coordinate
    for i, (x, y) in enumerate(waypoints):
        #Create the visual, the body and then some text

        visual =p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=GOAL_RADIUS,
            length=0.02,
            rgbaColor=[1,.75,0,1]
        )

        p.createMultiBody(
            bassMass= 0
            baseVisualShapeIndex = visual,
            basePosition + [x,y,0.015]
        )
        p.addUserBebugText(
            f"Target {i+1}",
            [x,y,0.45],
            textColorRGB=[.5,.3,0]
            textSize=1.5
        )
# ============================================================
# Set up PyBullet simulation
# ============================================================

client = p.connect(p.GUI)

if client < 0:
    raise RuntimeError("Could not open the PyBullet GUI")

p.configureDebugVisualizer(
    p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0
)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

p.loadURDF("plane.urdf")

robot_id = p.loadURDF(
    "husky/husky.urdf",
    [0, 0, 0.2]
)

##
##
# Display all destinations.
add_target_markers()

p.resetDebugVisualizerCamera(
    cameraDistance=14,
    cameraYaw=45,
    cameraPitch=-65,
    cameraTargetPosition=[3, 4, 0]
)

print("AUTONOMOUS HUSKY: MULTIPLE DESTINATIONS")
print(f"Total destinations: {len(waypoints)}")
print("Press Q to quit.")
print()
print(f"Navigating to TARGET 1: ({TARGET_X}, {TARGET_Y})")

last_report = time.monotonic()

mission_complete = False


# ============================================================
# Main autonomous navigation loop
# ============================================================

while p.isConnected():

    keys = p.getKeyboardEvents()

    if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
        break

    # --------------------------------------------------------
    # STEP 3: Read robot position and heading
    # --------------------------------------------------------
    position, orientation = p.getBasePositionAndOrientation(robot_id)

    x,y = position[0], position[1]

    heading = p.getEulerFromQuaternion(orientation)[2]


    # --------------------------------------------------------
    # STEP 4: Calculate distance and direction
    # --------------------------------------------------------
    distance = math.hypot(
        TARGET_X -x,
        TARGET_Y - y
    )

    target_angle = math.atan2(
        TARGET_X -x,
        TARGET_Y - y
    )

    heading_error = angle_difference(
        target_angle,
        heading
    )

    # --------------------------------------------------------
    # NEW STEP 5: Determine whether a destination was reached
    # --------------------------------------------------------

    if distance < GOAL_RADIUS and not mission_complete:
        #stop robot and print a message
        stop_robot(robot_id)

        print()
        print(f"Target {current_target + 1} Reached")


        # Advance to the next destination. Increase target by 1
        current_target = current_target + 1

        # Check whether more destinations remain.
        if current_target < len(waypoints):
             #Get coordinates of next target and print next heading
            TARGET_X, TARGET_Y = waypoints[current_target]

            print(f"Navigating to Target")

        else:
            #Nothing extra needed in this else
            # All destinations have been visited.
            mission_complete = True

            stop_robot(robot_id)

            print()
            print("==============================")
            print("       MISSION COMPLETE!")
            print("==============================")

    # --------------------------------------------------------
    # STEP 6: Autonomous navigation decisions
    # --------------------------------------------------------

    elif not mission_complete:
        #If there are more destinations, navigate to it
        #If heading_error is greater than tolerance turn left
        if heading_error > HEADING_TOLERANCE:
            turn_left(robot_id)
        
        #elIf destination is to the right, turn right
        elif heading_error < -HEADING_TOLERANCE:
            turn_right(robot_id)
        
        #else move forward
        else:
            move_forward(robot_id)




    # --------------------------------------------------------
    # Report robot position every 2 seconds
    # --------------------------------------------------------

    if not mission_complete:

        if time.monotonic() - last_report > 2:

            print(
                f"Position ({x:.1f}, {y:.1f}), "
                f"distance={distance:.2f} m, "
                f"target={current_target + 1}"
            )

            last_report = time.monotonic()


    # --------------------------------------------------------
    # Continue simulation
    # --------------------------------------------------------

    p.stepSimulation()

    time.sleep(1 / 240)


# ============================================================
# Clean shutdown
# ============================================================

if p.isConnected():

    stop_robot(robot_id)

    p.disconnect()