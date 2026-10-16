"""
CDS215 Week 8 Friday: Autonomous Husky Multi-Destination Challenge
Instructor Solution

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

#This determines how close the Husky must get to a destination before considering it reached.
GOAL_RADIUS = 0.65
SPEED = 7.0
#A slower turning speed gives the controller a better opportunity to align the robot before accelerating forward.
TURN_SPEED = 4.0
#This determines how accurately the Husky must face the destination before moving forward in radians. according to its yaw (z axis)
HEADING_TOLERANCE = 0.14 #in radians about 8 degrees

LEFT_WHEELS = [2, 4]
RIGHT_WHEELS = [3, 5]


# ============================================================
# NEW STEP 1A: Create a list of destinations
# ============================================================

# Instead of having just one destination, we now want our
# Husky to visit several destinations automatically.
#
# We create a LIST called waypoints.
#
# Each item in the list is a TUPLE containing two numbers:
# the X coordinate and the Y coordinate of a destination.
#
# The robot should visit these locations IN ORDER.

waypoints = [
    (4, 2),     # First destination
    (6, 7),     # Second destination
    (2, 8),      # Third destination
    (5, 10),     # Forth destination
    (7, 12),     # Fifth destination
]


# We need a way to remember which destination the robot
# is currently trying to reach.
#
# Lists start counting at index 0, not 1.
#
# Therefore:
#   waypoints[0] is (4, 2)
#   waypoints[1] is (6, 7)
#   waypoints[2] is (2, 8)
#
# We start with index 0 because that is our first destination.

current_target = 0


# Get the X and Y coordinates of our current destination.
#
# Since current_target = 0, this retrieves waypoints[0].
#
# In other words:
#   TARGET_X = 4
#   TARGET_Y = 2
#
# Python automatically unpacks the tuple into two variables.
#
# Later, we will update these variables whenever the robot
# reaches a destination.

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


# This function calculates the difference between the direction
# the robot SHOULD face and the direction it IS facing.
#
# It also handles the special case where angles wrap around
# from +180 degrees to -180 degrees.
#
# The result tells us how far and in which direction
# the robot needs to turn.

def angle_difference(target_angle, current_angle):
    return math.atan2(
        math.sin(target_angle - current_angle),
        math.cos(target_angle - current_angle)
    )


# ============================================================
# NEW STEP 2: Display all destinations
# ============================================================

# In our original program, we displayed only ONE target.
#
# Now we want to display a gold circle for EVERY destination.
#
# We use a loop to go through the entire waypoints list.

def add_target_markers():

    # enumerate() allows us to loop through a list while also
    # keeping track of the INDEX of each item.
    #
    # Each waypoint contains an X and Y coordinate.
    #
    # On each iteration:
    #
    #   i = the index of the waypoint
    #   x = the X coordinate
    #   y = the Y coordinate
    #
    # For our three destinations:
    #
    #   Iteration 1: i=0, x=4, y=2
    #   Iteration 2: i=1, x=6, y=7
    #   Iteration 3: i=2, x=2, y=8
    #
    # Notice that Python automatically unpacks (x, y).

    for i, (x, y) in enumerate(waypoints):

        # Create a gold cylinder to mark this destination.
        #
        # This is the same type of marker we used
        # in our original single-destination program.

        visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=GOAL_RADIUS,
            length=0.02,
            rgbaColor=[1, 0.75, 0.0, 1]
        )

        # Place the gold marker at this waypoint's coordinates.
        #
        # Notice that we use x and y from the loop.
        #
        # Every time the loop repeats, these values change
        # to the coordinates of the next destination.

        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual,
            basePosition=[x, y, 0.015]
        )

        # Add a label above each destination.
        #
        # We use i + 1 because Python indexes begin at zero.
        #
        # Without +1, our labels would be:
        #   TARGET 0
        #   TARGET 1
        #   TARGET 2
        #
        # With +1, our labels become:
        #   TARGET 1
        #   TARGET 2
        #   TARGET 3

        p.addUserDebugText(
            f"TARGET {i + 1}",
            [x, y, 0.45],
            textColorRGB=[0.5, 0.3, 0],
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


# Call our new function to draw all three destination markers.
#
# Because the function contains a loop, it automatically
# creates a marker for every waypoint in our list.

add_target_markers()


p.resetDebugVisualizerCamera(
    cameraDistance=14,
    cameraYaw=45,
    cameraPitch=-65,
    cameraTargetPosition=[3, 4, 0]
)

print("AUTONOMOUS HUSKY: MULTIPLE DESTINATIONS")

# len() tells us how many items are in the waypoints list.
#
# For our example, len(waypoints) returns 3.
#
# If we later add more destinations, this automatically
# reports the correct number.

print(f"Total destinations: {len(waypoints)}")

print("Press Q to quit.")
print()

print(f"Navigating to TARGET 1: ({TARGET_X}, {TARGET_Y})")

last_report = time.monotonic()


# NEW:
# This Boolean variable tells us whether the robot
# has completed its entire mission.
#
# False means there are still destinations to visit.
# True means every destination has been reached.
#
# We need this variable because the main simulation loop
# will continue running even after the mission is complete.

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

    x, y = position[0], position[1]

    heading = p.getEulerFromQuaternion(orientation)[2]


    # --------------------------------------------------------
    # STEP 4: Calculate distance and direction
    # --------------------------------------------------------

    # This is the SAME navigation mathematics we used
    # in our original autonomous Husky program.
    #
    # The important difference is that TARGET_X and TARGET_Y
    # can now change as the robot visits different waypoints.
    #
    # We do not need to rewrite our navigation algorithm!
    #
    # The robot simply navigates toward whichever destination
    # is currently stored in TARGET_X and TARGET_Y.

    #How far away is the destination? Straight line distance
    distance = math.hypot(
        TARGET_X - x,
        TARGET_Y - y
    )

    #Which direction is the destination? uses the difference in Y coordinates and X coordinates to calculate the angle in radians
    target_angle = math.atan2(
        TARGET_Y - y,
        TARGET_X - x
    )

    #How much does the robot need to turn? target angle minus current heading
    heading_error = angle_difference(
        target_angle,
        heading
    )


    # --------------------------------------------------------
    # NEW STEP 5: Determine whether a destination was reached
    # --------------------------------------------------------

    # First, check whether the robot is close enough
    # to the CURRENT destination.
    #
    # distance < GOAL_RADIUS means the robot has reached
    # the gold circle around its current target.
    #
    # We also check "not mission_complete" because
    # we only want to process destinations while there
    # are still destinations left to visit.

    if distance < GOAL_RADIUS and not mission_complete:

        # Stop the robot when it reaches the destination.
        stop_robot(robot_id)

        # Display which destination we just reached.
        #
        # Again, we use +1 because list indexes start at zero.

        print()
        print(f"TARGET {current_target + 1} REACHED!")


        # ----------------------------------------------------
        # NEW: Advance to the next destination
        # ----------------------------------------------------

        # This is one of the most important new commands.
        #
        # We increase current_target by 1.
        #
        # For example:
        #
        #   Before: current_target = 0
        #   After:  current_target = 1
        #
        # The robot has finished waypoint[0] and
        # now needs to navigate toward waypoint[1].
        #
        # This is equivalent to:
        # current_target = current_target + 1

        current_target += 1


        # ----------------------------------------------------
        # NEW: Check whether another destination exists
        # ----------------------------------------------------

        # len(waypoints) gives the number of destinations.
        #
        # In our example:
        # len(waypoints) = 3
        #
        # Valid indexes are:
        #   0, 1, and 2
        #
        # If current_target is LESS than 3,
        # another destination exists.
        #
        # If current_target becomes 3, we have visited
        # all destinations and should finish the mission.
        #
        # This check is important because trying to access
        # waypoints[3] would cause an IndexError.

        if current_target < len(waypoints):


            # ------------------------------------------------
            # NEW: Get coordinates of the next destination
            # ------------------------------------------------

            # Retrieve the coordinates of the next waypoint.
            #
            # Suppose current_target is now 1.
            #
            # Then:
            #   waypoints[1] returns (6, 7)
            #
            # Python unpacks that tuple:
            #   TARGET_X = 6
            #   TARGET_Y = 7
            #
            # On the next iteration of our main loop,
            # the robot will automatically calculate
            # the distance and direction to this NEW target.
            #
            # Notice that we don't need to change any
            # of the original navigation mathematics!

            TARGET_X, TARGET_Y = waypoints[current_target]


            # Tell the user where the robot is heading next.

            print(
                f"Navigating to TARGET {current_target + 1}: "
                f"({TARGET_X}, {TARGET_Y})"
            )


        else:

            # ------------------------------------------------
            # NEW: Mission complete
            # ------------------------------------------------

            # If we reach this else statement,
            # current_target is no longer a valid index.
            #
            # That means the robot has visited EVERY
            # destination in the waypoints list.
            #
            # Set mission_complete to True.
            #
            # The robot will no longer receive movement
            # commands from the navigation decisions below.

            mission_complete = True

            # Stop the wheels.
            stop_robot(robot_id)

            print()
            print("==============================")
            print("       MISSION COMPLETE!")
            print("==============================")


    # --------------------------------------------------------
    # STEP 6: Autonomous navigation decisions
    # --------------------------------------------------------

    # This is the same turning and forward movement logic
    # from our original single-destination program.
    #
    # The difference is that we now use:
    #
    # elif not mission_complete:
    #
    # This means the robot should only continue navigating
    # if there are still destinations left to visit.
    #
    # If the robot just reached a waypoint, the IF block
    # above handles that event and selects the next target.
    #
    # On the next pass through the loop, the robot will
    # calculate a new distance and heading error.

    elif not mission_complete:

        # If the destination is to the left, turn left.

        if heading_error > HEADING_TOLERANCE:
            turn_left(robot_id)

        # If the destination is to the right, turn right.

        elif heading_error < -HEADING_TOLERANCE:
            turn_right(robot_id)

        # Otherwise, the robot is facing close enough
        # to the destination, so move forward.

        else:
            move_forward(robot_id)


    # --------------------------------------------------------
    # Report robot position every 2 seconds
    # --------------------------------------------------------

    # Only print navigation information while the robot
    # is still working on its mission.

    if not mission_complete:

        if time.monotonic() - last_report > 2:

            # Notice that we now include the current target.
            #
            # This makes it easier to see which waypoint
            # the robot is navigating toward.

            print(
                f"Position ({x:.1f}, {y:.1f}), "
                f"distance={distance:.2f} m, "
                f"target={current_target + 1}"
            )

            last_report = time.monotonic()


    # --------------------------------------------------------
    # Continue simulation
    # --------------------------------------------------------

    # Even after the mission is complete, PyBullet continues
    # running so we can view the robot at its final destination.
    #
    # The simulation advances by 1/240 of a second each time
    # this command executes.
    #
    # The main loop repeats, continually reading the robot's
    # position and making navigation decisions until the
    # mission is complete.

    p.stepSimulation()

    time.sleep(1 / 240)


# ============================================================
# Clean shutdown
# ============================================================

if p.isConnected():

    stop_robot(robot_id)

    p.disconnect()