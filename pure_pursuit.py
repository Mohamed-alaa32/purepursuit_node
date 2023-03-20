#!/usr/bin/env python3
"""
Pure Pursuit Controller
"""
import math
from dataclasses import dataclass
from typing import List, Tuple
import rospy

# import rospy
import numpy as np
from geometry_msgs.msg import Pose

# from ackermann_msgs.msg import AckermannDriveStamped
# import asurt_msgs.msg as asurt_msgs
# from std_msgs.msg import Path
#from nav_msgs.msg import Path

DT = 0.1  # rospy.get_param("/time_step") # [s] time step
BASEWIDTH = 2.9  # rospy.get_param("/base_width") # [m] car length
GAINLOOKAHEAD = 0.5  # rospy.get_param("/gains/look_forward") # look forward gain
LOOKAHEADCONSTANT = 2.0  # rospy.get_param("/look_ahead_constant") # look ahead constant


@dataclass
class Position:
    """
    data class to store position of the vehicle
    """

    x: float
    y: float


class State:
    """
    state of the vehicle received from SLAM
    """

    def __init__(self, position: Position, yaw: float, currentSpeed: float = 0.0) -> None:
        """
        parameters
        ----------
        x : float
            x coordinate of the vehicle

        y : float
            y coordinate of the vehicle

        yaw : float
            yaw of the vehicle
        
        currentSpeed : float
            current speed of the vehicle

        rearX : float
            x coordinate of the rear of the vehicle

        rearY : float
            y coordinate of the rear of the vehicle
        """
        self.position: Position = Position(position.x, position.y)
        self.oldPosition:Position = Position(position.x, position.y)
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.rearX = self.position.x - ((BASEWIDTH / 2) * math.cos(self.yaw))
        self.rearY = self.position.y - ((BASEWIDTH / 2) * math.sin(self.yaw))
    def update(self, currentState: Pose) -> None:
        """
        update the state of the vehicle to the new state

        Parameters
        ----------
        currentState : Pose
            new state of the vehicle
        """
        # position = Position(currentState.position.x, currentState.position.y, oldX, self.oldY)
        self.oldPosition.x = self.position.x
        self.oldPosition.y = self.position.y
        self.position.x = currentState.position.x
        self.position.y = currentState.position.y
        self.yaw = currentState.orientation.z
        self.currentSpeed = currentState.orientation.x #(
           # math.sqrt((math.pow(self.position.x - self.oldPosition.x, 2) + math.pow(self.position.y - self.oldPosition.y, 2))) / DT
        #)
        self.rearX = self.position.x - ((BASEWIDTH / 2) * math.cos(self.yaw))
        self.rearY = self.position.y - ((BASEWIDTH / 2) * math.sin(self.yaw))

    def calcDistance(self, pointX: float, pointY: float) -> float:
        """
        calculate the distance between the rear of the vehicle and a point

        Parameters
        ----------
        pointX : float
            x coordinate of the point

        pointY : float
            y coordinate of the point

        Returns
        -------
        distance : float
            distance between the rear of the vehicle and the point

        """
        distanceX:float = self.rearX - pointX
        distanceY:float = self.rearY - pointY
        distance:float = math.hypot(distanceX, distanceY)
        
        return distance


class States:
    """
    Class to store new state to a list of states of the vehicle at each time step
    """

    def __init__(self) -> None:
        """
        Parameters
        ----------
        xList : List[float]
            list of x coordinates that the vehicle has passed through
        
        yList : List[float]
            list of y coordinates that the vehicle has passed through

        yaws : List[float]
            list of yaw angles that the vehicle has passed through
        
        currentSpeeds : List[float]
            list of current speeds that the vehicle has passed through

        """
        self.yList: List[float] = []
        self.xList: List[float] = []
        self.yaws: List[float] = []
        self.currentSpeeds: List[float] = []
        

    def add(self, state: State) -> None:
        """
        Add each state element to it's corrosponding list

        Parameters
        ----------
        state : State
            state of the vehicle at each time step

        """
        self.xList.append(state.x)
        self.yList.append(state.y)
        self.yaws.append(state.yaw)
        self.currentSpeeds.append(state.currentSpeed)

    def statesCounter(self) -> int:
        """
        Count the number of states that the vehicle has passed through so far

        Returns
        -------
        statesNumber : int
            number of elements in any of the lists
        """
        assert len(self.xList) == len(self.yList) == len(self.yaws) == len(self.currentSpeeds)
        statesNumber: int = len(self.xList)
        return statesNumber


class WayPoints:
    """
    Class to store new waypoints to a list of waypoints and search for the suitable target point
    to follow with the pure pursuit algorithm
    """

    def __init__(self) -> None:
        """
        Parameters
        ----------
        xList : List[float]
            list of x coordinates of the waypoints
        
        yList : List[float]
            list of y coordinates of the waypoints

        oldNearestPointIndex : int
            index of the nearest point to the vehicle at the previous time step

        """
        self.xList: List[float] = []
        self.yList: List[float] = []
        self.oldNearestPointIndex:int = 0
        self.firstLoop:bool = False

    def add(self, waypoints: Pose) -> None:
        """
        Add each waypoint element to it's corrosponding list

        Parameters
        ----------
        waypoints : Pose
            waypoint of the vehicle received from the path planner
        """
        
        self.xList.append(waypoints.position.x)  # poses[-1].position.x)
        self.yList.append(waypoints.position.y)  # poses[-1].position.y)

    def searchTargetIndex(self, state: State) -> Tuple[int, float,str]:
        """
        Search for the nearest point to the vehicle and calculate the distance between the vehicle

        Parameters
        ----------
        state : State
            current state of the vehicle

        Returns
        -------
        ind : int
            target point index choosen to follow from the waypoints list
        
        lookahead : float
            lookahead distance to the target point
        """
        # To speed up nearest point search, doing it at only first time
        clearance = 0.6
        message = "no message"
        if self.firstLoop is False:
            # search nearest point index
            distanceX = [state.rearX - icx for icx in self.xList]
            distanceY = [state.rearY - icy for icy in self.yList]
            distance = np.hypot(distanceX, distanceY)
            if len(distance) != 0:
                ind: int = int(np.argmin(distance))
                self.oldNearestPointIndex = ind
                self.firstLoop = True
            # else: # if there is no waypoints bas hatetshal w abadlha be eny ma3mlsh elfunction 8er ama at2aked en feh waypoints
            #     ind = 0
            #     self.oldNearestPointIndex = ind
            #     self.firstLoop = True
        else:
            ind = self.oldNearestPointIndex
            distanceThisIndex = state.calcDistance(self.xList[ind], self.yList[ind])
            while ind < len(self.xList) - 1:
                distanceNextIndex = state.calcDistance(self.xList[ind + 1], self.yList[ind + 1])
                if distanceThisIndex < clearance:
                    ind = ind + 1
                    
                    #rospy.loginfo_once("ind is increased")
                elif distanceNextIndex > clearance + 0.2:
                    ind = ind -1
                    #rospy.loginfo("ind is decreased")
                    break
                else:
                    #lookAhead = "while loop broken"
                    message = "while loop broken"
                    break
                # if distanceThisIndex < distanceNextIndex:
                #     break
                # ind = ind + 1 if (ind + 1) < len(self.xList) else ind
                distanceThisIndex = distanceNextIndex
            self.oldNearestPointIndex = ind

        lookAhead: float = GAINLOOKAHEAD * state.currentSpeed + LOOKAHEADCONSTANT
        # update look ahead distance
        # search look ahead target point index
        # while lookAhead > state.calcDistance(self.xList[ind], self.yList[ind]):
        #     if (ind + 1) >= len(self.xList):
        #         break  # not exceed goal
        #     ind += 1
        print("search Target index function is called")
        return ind, lookAhead, message  # 3ayz a2asem de le 2 functions


def purepursuitSteercontrol(state: State, trajectory: WayPoints, pind: int) -> Tuple[float, int, str]:
    """
    Calculate the steering angle to follow the target point

    Parameters
    ----------
    state : State
        current state of the vehicle
    
    trajectory : WayPoints
        list of waypoints to follow

    pind : int
        index of the nearest point to the vehicle at the previous time step

    
    Returns
    -------
    delta : float
        steering angle to follow the target point

    ind : int
        index of the target point
    """
    ind, lookAhead, message = trajectory.searchTargetIndex(state)
    trajX: float = 0
    trajY: float = 0
    if pind >= ind:
        ind = pind
        #print("went to first if")
    if ind < len(trajectory.xList):
        trajX = trajectory.xList[ind]
        trajY = trajectory.yList[ind]
        #print("went to second if")
    else:  # toward goal
        trajX = trajectory.xList[-1] #da lw closed loop, msh m7tagha lw kol lap elwaypoints bttgaded
        trajY = trajectory.yList[-1]
        ind = len(trajectory.xList) - 1
        #print("went to third if")
    alpha: float = math.atan2(trajY - state.rearY, trajX - state.rearX) - state.yaw
    print("purepursuit function called")
    print("alpha: ", alpha)
    #alpha  = -math.pi/2
    lookAhead = 5
    delta: float = math.atan2(2.0 * BASEWIDTH * math.sin(alpha) / lookAhead, 1.0) # must be between -60 and 60 , lookahead must be = 0.86 * base width
    deltaDegree = math.degrees(delta)
    if abs(deltaDegree) < 25:
        ind = ind + 1 if (ind + 1) < len(trajectory.xList) else ind
    print("lookAhead: ", lookAhead)
    print("delta: ", delta)
    print("deltaDegree: ", deltaDegree)
    print("\n")
    return delta, ind, message
