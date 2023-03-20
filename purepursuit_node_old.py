"""
Initilization Pure Pursuit node for vehicle control
"""
import rospy
import math
# from purepursuit import State, WayPoints, States, purepursuitSteercontrol, proportionalControl
# from pure_pursuit import purepursuitSteercontrol
from asurt_msgs.msg import NodeStatus
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose
from pure_pursuit import WayPoints, State, purepursuitSteercontrol, proportionalControl, targetSpeedCalc, States, Position
# from pid_controller import proportionalControl, targetSpeedCalc
from nav_msgs.msg import Path
import matplotlib.pyplot as plt


# acc = proportionalControl(102.0, 5.0)
# print(acc)
# state  = State(0,0,0)
# state.x = 1.0
# print(state.x)


def main():
    """
    main function for pure pursuit vehicle control
    """
    rospy.init_node("purepursuit_controller", anonymous=True)

    # message = NodeStatus()
    # message.status = #starting
    # rospy.wait_for_message("/waypoints", Pose)
    # rospy.wait_for_message("/state", Pose)

    controlActionPub = rospy.Publisher("/control_actions", AckermannDrive, queue_size=10)
    waypoints = WayPoints()
    waypoints.xList =[0.0]
    waypoints.yList = [0.0]
    position = Position(0.0, 0.0)
    #alpha = math.atan2(waypoints.yList[targetInd] - state.rearY, waypoints.xList[targetInd] - state.rearX)
    state = State(position , 0.0)
    rospy.Subscriber("/state", Pose, callback=state.update)
    rospy.Subscriber("/waypoints", Pose, callback=waypoints.add)

    controlAction = AckermannDrive()
    # message.status = #ready
    rate = rospy.Rate(10)
    targetInd = 0
    #alpha = math.atan2(waypoints.yList[targetInd] - state.rearY, waypoints.xList[targetInd] - state.rearX)
    while not rospy.is_shutdown():  # and using_pure_pursuit = true
        #wait for atleast 1 waypoint
        
        # targetInd, _ = waypoints.searchTargetIndex(state)
        #ind , message = waypoints.searchTargetIndex(state)
        delta, targetInd, message = purepursuitSteercontrol(
            state, waypoints, targetInd
        )  # lateral controller
        targetSpeed = targetSpeedCalc(delta)
        #rospy.loginfo("test")
        acc = proportionalControl(targetSpeed, state.currentSpeed)  # longitudinal controller
        deltaDegree = math.degrees(delta)
        controlAction.acceleration = acc  # proportionalControl(targetSpeed, state.currentSpeed)
        controlAction.steering_angle = delta
        controlAction.jerk = targetInd
        controlAction.speed = deltaDegree
        controlAction.steering_angle_velocity = waypoints.yList[targetInd]
        rospy.loginfo(message)
        # clearance = state.calcDistance(waypoints.X_coordinates[-1], waypoints.Y_coordinates[-1])
        # if clearance <= 1.4:
        #     # goal_reached = True
        #     print("Goal Reached")
        #     controlAction.acceleration = 0.0
        #     controlAction.steering_angle = 0.0
        testname = "add clearance if statement"
        controlActionPub.publish(controlAction)
        plot(waypoints, state, testname, targetInd)
        # message.status = #running

        rate.sleep()

    # assert lastIndex >= targetInd, "Cannot reach goal"
def plot(waypoints, state, name, target_ind):
    plt.plot(waypoints.xList, waypoints.yList, "-r", label="course")
    #plt.plot(waypoints.xList[targetInd], waypoints.yList[targetInd], "xg", label="target")
    plt.plot(state.position.x, state.position.y, "ob", label="state")
    plt.grid(True)
    plt.plot(waypoints.xList[target_ind], waypoints.yList[target_ind], "xg", label="target")
    plt.axis("equal")
    plt.title(name)
    #plt.legend()
    plt.pause(0.001)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# target_course = WayPoints(X_coordinates, Y_coordinates, Z_coordinates)
#     targetInd, _ = target_course.search_target_index(state)
# <rosparam file="$(find pure_pursuit)/tests/parameters.yaml" />
