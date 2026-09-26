#! /usr/bin/env python3

import numpy as np
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget, StatusText
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from geographic_msgs.msg import GeoPointStamped

from Utils import *


POSITION_IGNORE = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ
VELOCITY_IGNORE = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ
ACCELERATION_IGNORE = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
YAW_IGNORE = PositionTarget.IGNORE_YAW
YAW_RATE_IGNORE = PositionTarget.IGNORE_YAW_RATE

POSITION_YAW = VELOCITY_IGNORE + ACCELERATION_IGNORE + YAW_RATE_IGNORE
VELOCITY_YAW = POSITION_IGNORE + ACCELERATION_IGNORE + YAW_RATE_IGNORE
ACCELERATION_YAW = POSITION_IGNORE + VELOCITY_IGNORE + YAW_RATE_IGNORE
TRAJECTORY_YAW = YAW_RATE_IGNORE

POSITION_YAW_RATE = VELOCITY_IGNORE + ACCELERATION_IGNORE + YAW_IGNORE
VELOCITY_YAW_RATE = POSITION_IGNORE + ACCELERATION_IGNORE + YAW_IGNORE
ACCELERATION_YAW_RATE = POSITION_IGNORE + VELOCITY_IGNORE + YAW_IGNORE

POSITION_SETPOINT_ALL_IGNORE = POSITION_IGNORE + VELOCITY_IGNORE + ACCELERATION_IGNORE + YAW_IGNORE + YAW_RATE_IGNORE

ANGLE_RATE_IGNORE = AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_YAW_RATE
ATTITUDE_IGNORE = ANGLE_RATE_IGNORE + AttitudeTarget.IGNORE_ATTITUDE

ATTITUDE_SETPOINT_ALL_IGNORE = ATTITUDE_IGNORE + ANGLE_RATE_IGNORE + AttitudeTarget.IGNORE_THRUST


class P230():
    def __init__(self, name='uav1', localOffsetENU=None) -> None:
        self.name = name
        self.configuredLocalOffsetENU = np.array(localOffsetENU if localOffsetENU is not None else [0.0, 0.0, 0.0], dtype=float)
        self.localOffsetENU = np.array(self.configuredLocalOffsetENU, dtype=float)

        self.armService = rospy.ServiceProxy(f'/{name}/mavros/cmd/arming', CommandBool)
        self.setModeService = rospy.ServiceProxy(f'/{name}/mavros/set_mode', SetMode)
        self.takeoffService = rospy.ServiceProxy(f'/{name}/mavros/cmd/takeoff', CommandTOL)
        self.landService = rospy.ServiceProxy(f'/{name}/mavros/cmd/land', CommandTOL)

        self.meState = State()

        self.rawPositionENU = np.zeros(3)
        self.rawVelocityENU = np.zeros(3)
        self.mePositionENU = np.zeros(3)
        self.meVelocityENU = np.zeros(3)
        self.meSpeed = 0
        self.meAccelerationFLU = np.zeros(3)
        self.meAccelerationENU = np.zeros(3)
        self.meQuaternionENU = np.array([0, 0, 0, 1])
        self.rawQuaternionENU = np.array([0, 0, 0, 1])
        self.meRPYRadENU = np.zeros(3)
        self.meRPYRadNED = np.zeros(3)
        self.meRPYDegENU = np.zeros(3)

        self.stateSub = rospy.Subscriber(f'/{name}/mavros/state', State, self.state_cb)
        self.statusTextSub = rospy.Subscriber(f'/{name}/mavros/statustext/recv', StatusText, self.status_text_cb)
        self.localPosSub = rospy.Subscriber(f'/{name}/mavros/local_position/pose', PoseStamped, self.local_pos_cb)
        self.velSub = rospy.Subscriber(f'/{name}/mavros/local_position/velocity_local', TwistStamped, self.vel_cb)
        self.imuSub = rospy.Subscriber(f'/{name}/mavros/imu/data', Imu, self.imu_cb)
        self.gpsOriginPub = rospy.Publisher(f'/{name}/mavros/global_position/set_gp_origin', GeoPointStamped,queue_size=1)
        self.gpsOriginSub = rospy.Subscriber(f'/{name}/mavros/global_position/gp_origin', GeoPointStamped, self.origin_cb)
        
        self.gpsOriginZurichIrchelPark = GeoPointStamped()
        self.gpsOriginZurichIrchelPark.position.latitude = 47.397742
        self.gpsOriginZurichIrchelPark.position.longitude = 8.545594
        self.gpsOriginZurichIrchelPark.position.altitude = 535.4
        self.gpsOriginZurichIrchelPark.header.frame_id = "base_link"
        self.gpsOrigin = self.gpsOriginZurichIrchelPark

        self.meGpsOrigin = GeoPointStamped()

        self.setpoint = PositionTarget()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint.type_mask = POSITION_YAW_RATE
        self.setpoint.position.x = 0
        self.setpoint.position.y = 0
        self.setpoint.position.z = 0
        self.setpointPub = rospy.Publisher(f'/{name}/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        self.attitudeSetpoint = AttitudeTarget()
        self.attitudeSetpoint.type_mask = ATTITUDE_IGNORE
        self.attitudeSetpoint.orientation.x = 0
        self.attitudeSetpoint.orientation.y = 0
        self.attitudeSetpoint.orientation.z = 0
        self.attitudeSetpoint.orientation.w = 1
        self.attitudeSetpointPub = rospy.Publisher(f'/{name}/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        self.velMax = np.array([1, 1, 1])
        self.accMax = np.array([1, 1, 1])
        self.yawRateRadMax = 1
        self.yawRadNED = 0
        self.rollDegMax = 10.0
        self.pitchDegMax = 10.0
        self.rpRadMax = np.deg2rad(np.array([self.rollDegMax, self.pitchDegMax]))

        self.rollOffsetRad = 0.0
        self.pitchOffsetRad = 0.0
        self.lastControlPrintTime = 0.0
        self.controlPrintPeriod = 1.0
        self.lastOffboardRequestTime = 0.0
        self.lastArmRequestTime = 0.0
        self.lastFcuNotReadyPrintTime = 0.0
        self.fcuNotReadyPrintPeriod = 1.0
        self.lastStatusText = ''
        self.lastStatusTextSeverity = None
        self.lastStatusTextTime = 0.0
        self.externalWorldStateEnabled = False

        self.fcu_url = None
        while not self.fcu_url:
            self.fcu_url = rospy.get_param(f'/{name}/mavros/fcu_url', None)
            if not self.fcu_url:
                rospy.logwarn_throttle(5.0, f'Waiting for /{name}/mavros/fcu_url')
            time.sleep(1)
        print(f'Connected to FCU at {self.fcu_url}')
        if 'udp' in self.fcu_url:
            self.mode = 'sim'
        elif 'dev' in self.fcu_url:
            self.mode = 'real'
        else:
            self.mode = 'unknown'
            raise ValueError('FCU URL parse failed')

        self.hoverThrottle = 0.4

        self.kp = 0.5

        self.positionThreshold = 0.5 if self.mode == 'sim' else 0.1
        self.speedThreshold = 0.4 if self.mode == 'sim' else 0.1 

        print('P230 node initialized')

    def state_cb(self, msg):
        self.meState = msg

    def status_text_cb(self, msg):
        text = str(msg.text).strip()
        self.lastStatusText = text
        self.lastStatusTextSeverity = msg.severity
        self.lastStatusTextTime = time.time()
        lowerText = text.lower()
        if any(keyword in lowerText for keyword in ('arm', 'preflight', 'reject', 'denied', 'fail')):
            print(f'PX4 status text ({self.name}): severity={msg.severity}, text={text}')

    def isConnected(self):
        return bool(self.meState.connected)

    def hasMode(self):
        return bool(self.meState.mode)

    def fcuReadyForControl(self):
        return self.isConnected() and self.hasMode()

    def printFcuNotReady(self, action):
        now = time.time()
        if now - self.lastFcuNotReadyPrintTime < self.fcuNotReadyPrintPeriod:
            return
        self.lastFcuNotReadyPrintTime = now
        print(
            f'Waiting for FCU before {action}: '
            f'connected={self.meState.connected}, mode={self.meState.mode or "<empty>"}'
        )

    def local_pos_cb(self, msg):
        rawPositionENU = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.rawPositionENU = rawPositionENU
        self.rawQuaternionENU = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ])
        if self.externalWorldStateEnabled:
            return

        self.mePositionENU = rawPositionENU + self.localOffsetENU
        self.meQuaternionENU = np.array(self.rawQuaternionENU, dtype=float)
        self.meRPYRadENU = quaternion2euler(self.meQuaternionENU)
        self.meRPYRadNED = rpyENU2NED(self.meRPYRadENU)
        self.meRPYDegENU = np.rad2deg(self.meRPYRadENU)

    def configuredMavrosPositionENU(self):
        return self.rawPositionENU + self.configuredLocalOffsetENU

    def applyWorldPositionCorrectionENU(self, worldPositionENU):
        worldPositionENU = np.asarray(worldPositionENU, dtype=float)
        if not np.all(np.isfinite(worldPositionENU)):
            return
        self.localOffsetENU = worldPositionENU - self.rawPositionENU
        self.mePositionENU = np.array(worldPositionENU, dtype=float)

    def setExternalWorldStateEnabled(self, enabled):
        self.externalWorldStateEnabled = bool(enabled)

    def applyWorldStateCorrectionENU(self, worldPositionENU, worldVelocityENU, worldQuaternionENU):
        self.applyWorldPositionCorrectionENU(worldPositionENU)

        worldVelocityENU = np.asarray(worldVelocityENU, dtype=float)
        if np.all(np.isfinite(worldVelocityENU)):
            self.meVelocityENU = np.array(worldVelocityENU, dtype=float)
            self.meSpeed = np.linalg.norm(self.meVelocityENU)

        worldQuaternionENU = np.asarray(worldQuaternionENU, dtype=float)
        if worldQuaternionENU.shape == (4,) and np.all(np.isfinite(worldQuaternionENU)):
            quaternionNorm = np.linalg.norm(worldQuaternionENU)
            if quaternionNorm > 1e-6:
                self.meQuaternionENU = worldQuaternionENU / quaternionNorm
                self.meRPYRadENU = quaternion2euler(self.meQuaternionENU)
                self.meRPYRadNED = rpyENU2NED(self.meRPYRadENU)
                self.meRPYDegENU = np.rad2deg(self.meRPYRadENU)

    def applyRawMavrosAttitudeENU(self):
        rawQuaternionENU = np.asarray(self.rawQuaternionENU, dtype=float)
        if rawQuaternionENU.shape != (4,) or not np.all(np.isfinite(rawQuaternionENU)):
            return

        quaternionNorm = np.linalg.norm(rawQuaternionENU)
        if quaternionNorm <= 1e-6:
            return

        self.meQuaternionENU = rawQuaternionENU / quaternionNorm
        self.meRPYRadENU = quaternion2euler(self.meQuaternionENU)
        self.meRPYRadNED = rpyENU2NED(self.meRPYRadENU)
        self.meRPYDegENU = np.rad2deg(self.meRPYRadENU)

    def vel_cb(self, msg):
        self.rawVelocityENU = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
        if self.externalWorldStateEnabled:
            return

        self.meVelocityENU = np.array(self.rawVelocityENU, dtype=float)
        self.meSpeed = np.linalg.norm(self.meVelocityENU)

    def imu_cb(self, msg):
        self.meAccelerationFLU = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.meAccelerationENU = flu2enuRotationMatrix(self.meRPYRadENU[0], self.meRPYRadENU[1], self.meRPYRadENU[2]) @ self.meAccelerationFLU + np.array([0, 0, -GRAVITY])

    def origin_cb(self, msg):
        self.meGpsOrigin = msg

    def intoOffboardMode(self):
        try:
            offboardSetMode = SetModeRequest()
            offboardSetMode.custom_mode = 'OFFBOARD'
            response = self.setModeService.call(offboardSetMode)

            if self.meState.mode == 'OFFBOARD':
                return True

            if not response.mode_sent:
                rospy.logerr("into offboard mode failed: mode_sent=%s" % response.mode_sent)
                print(f'OFFBOARD request rejected: mode_sent={response.mode_sent}, current_mode={self.meState.mode}')
                return False

            statusText = ''
            if self.lastStatusText:
                age = time.time() - self.lastStatusTextTime
                statusText = (
                    f', last_status_text_age={age:.1f}s, '
                    f'last_status_text_severity={self.lastStatusTextSeverity}, '
                    f'last_status_text="{self.lastStatusText}"'
                )
            print(f'OFFBOARD request sent; waiting for state update, current_mode={self.meState.mode}{statusText}')
            return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    def requestOffboardIfNeeded(self, period=0.5):
        if self.meState.mode == 'OFFBOARD':
            return True
        if not self.fcuReadyForControl():
            self.printFcuNotReady('OFFBOARD request')
            return False
        now = time.time()
        if now - self.lastOffboardRequestTime < period:
            return False
        self.lastOffboardRequestTime = now
        return self.intoOffboardMode()

    def arm(self):
        try:
            armCmd = CommandBoolRequest()
            armCmd.value = True
            response = self.armService.call(armCmd)

            if not response.success and not self.meState.armed:
                rospy.logerr("arm failed: success=%s result=%s" % (response.success, response.result))
                statusText = ''
                if self.lastStatusText:
                    age = time.time() - self.lastStatusTextTime
                    statusText = (
                        f', last_status_text_age={age:.1f}s, '
                        f'last_status_text_severity={self.lastStatusTextSeverity}, '
                        f'last_status_text="{self.lastStatusText}"'
                    )
                print(
                    f'ARM request rejected: success={response.success}, '
                    f'result={response.result}, mode={self.meState.mode}{statusText}'
                )
                return False

            rospy.loginfo("arm successful!")
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    def armIfNeeded(self, period=1.0):
        if self.isArmed():
            return True
        if not self.fcuReadyForControl():
            self.printFcuNotReady('ARM request')
            return False
        now = time.time()
        if now - self.lastArmRequestTime < period:
            return False
        self.lastArmRequestTime = now
        return self.arm()

    def disarm(self):
        try:
            armCmd = CommandBoolRequest()
            armCmd.value = False
            response = self.armService.call(armCmd)

            if not response.success and self.meState.armed:
                rospy.logerr("disarm failed: success=%s result=%s" % (response.success, response.result))
                print(f'DISARM request rejected: success={response.success}, result={response.result}, mode={self.meState.mode}')
                return False

            rospy.loginfo("disarm successful!")
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

    def takeoff(self, height=1.0):
        self.takeoffCmd = CommandTOLRequest()
        self.takeoffCmd.altitude = height
        return self.takeoffService.call(self.takeoffCmd)

    def land(self):
        self.landCmd = CommandTOLRequest()
        return self.landService.call(self.landCmd)

    def isArmed(self):
       return self.meState.armed

    def mode(self):
        return self.meState.mode

    def sendHeartbeat(self):
        nowStamp = rospy.Time.now()
        self.setpoint.header.stamp = nowStamp
        self.attitudeSetpoint.header.stamp = nowStamp
        if self.attitudeSetpoint.type_mask == ANGLE_RATE_IGNORE:
            self.attitudeSetpointPub.publish(self.attitudeSetpoint)
        else:
            self.setpointPub.publish(self.setpoint)
        now = time.time()
        if now - self.lastControlPrintTime >= self.controlPrintPeriod:
            self.printControl()
            self.lastControlPrintTime = now

    def setPositionControlMode(self):
        self.setpoint.type_mask = POSITION_YAW
        self.attitudeSetpoint.type_mask = ATTITUDE_SETPOINT_ALL_IGNORE

    def setVelocityControlMode(self):
        self.setpoint.type_mask = VELOCITY_YAW
        self.attitudeSetpoint.type_mask = ATTITUDE_SETPOINT_ALL_IGNORE

    def setAccelerationControlMode(self):
        self.setpoint.type_mask = ACCELERATION_YAW
        self.attitudeSetpoint.type_mask = ATTITUDE_SETPOINT_ALL_IGNORE

    def setTrajectoryControlMode(self):
        self.setpoint.type_mask = TRAJECTORY_YAW
        self.attitudeSetpoint.type_mask = ATTITUDE_SETPOINT_ALL_IGNORE

    def setAttitudeControlMode(self):
        self.setpoint.type_mask = POSITION_SETPOINT_ALL_IGNORE
        self.attitudeSetpoint.type_mask = ANGLE_RATE_IGNORE

    def printControl(self):
        print('-' * 10 + 'Control' + '-' * 10)
        print('Control Mode: ', end='')
        if self.attitudeSetpoint.type_mask == ANGLE_RATE_IGNORE:
            print('Attitude')
            quat = self.attitudeSetpoint.orientation
            print(f'Quat: ({quat.x:.2f}, {quat.y:.2f}, {quat.z:.2f}, {quat.w:.2f})')
            print(f'Euler: {rpyString(quaternion2eulerXYZW(quat))}')
            print(f'Thrust: {self.attitudeSetpoint.thrust:.4f}')
        elif self.setpoint.type_mask == POSITION_YAW:
            print('Position & Yaw')
            print(f'Position: {pointString(self.setpoint.position)}')
            print(f'Yaw: {np.rad2deg(self.setpoint.yaw):.2f} deg')
        elif self.setpoint.type_mask == VELOCITY_YAW:
            print('Velocity & Yaw')
            print(f'Velocity: {pointString(self.setpoint.velocity)}')
            print(f'Yaw: {np.rad2deg(self.setpoint.yaw):.2f} deg')
        elif self.setpoint.type_mask == ACCELERATION_YAW:
            print('Acceleration & Yaw')
            print(f'Acceleration: {pointString(self.setpoint.acceleration_or_force)}')
            print(f'Yaw: {np.rad2deg(self.setpoint.yaw):.2f} deg')
        elif self.setpoint.type_mask == TRAJECTORY_YAW:
            print('Trajectory & Yaw')
            print(f'Position: {pointString(self.setpoint.position)}')
            print(f'Velocity: {pointString(self.setpoint.velocity)}')
            print(f'Acceleration: {pointString(self.setpoint.acceleration_or_force)}')
            print(f'Yaw: {np.rad2deg(self.setpoint.yaw):.2f} deg')
        elif self.setpoint.type_mask == POSITION_YAW_RATE:
            print('Position & Yaw Rate')
            print(f'Position: {pointString(self.setpoint.position)}')
            print(f'Yaw rate: {np.rad2deg(self.setpoint.yaw_rate):.2f} deg')
        elif self.setpoint.type_mask == VELOCITY_YAW_RATE:
            print('Velocity & Yaw Rate')
            print(f'Velocity: {pointString(self.setpoint.velocity)}')
            print(f'Yaw rate: {np.rad2deg(self.setpoint.yaw_rate):.2f} deg')
        elif self.setpoint.type_mask == ACCELERATION_YAW_RATE:
            print('Acceleration & Yaw Rate')
            print(f'Acceleration: {pointString(self.setpoint.acceleration_or_force)}')
            print(f'Yaw rate: {np.rad2deg(self.setpoint.yaw_rate):.2f} deg')
        else:
            print('Unknown')

    def printMe(self):
        print('-' * 10 + 'Me' + '-' * 10)
        print(f'FCU connected: {"YES" if self.meState.connected else "NO"}')
        print('Mode: ', self.meState.mode)
        print('Position ENU: ', arrayString(self.mePositionENU))
        print('Velocity ENU: ', arrayString(self.meVelocityENU))
        print(f'Speed: {self.meSpeed:.2f}')
        print('Acceleration FLU: ', arrayString(self.meAccelerationFLU))
        print('Acceleration ENU: ', arrayString(self.meAccelerationENU))
        print('Euler Deg ENU: ', rpyString(self.meRPYRadENU))
        print(f'Hover throttle: {self.hoverThrottle:.2f}')

    def distanceToPointENU(self, pointENU):
        return np.linalg.norm(self.mePositionENU - pointENU)

    def nearPositionENU(self, pointENU, tol=None):
        if tol is None:
            tol = self.positionThreshold
        return self.distanceToPointENU(pointENU) <= tol

    def nearSpeed(self, speed, tol=None):
        if tol is None:
            tol = self.speedThreshold
        return abs(speed - self.meSpeed) <= tol

    def aboveHeight(self, height):
        return self.mePositionENU[2] >= height

    def belowHeight(self, height):
        return self.mePositionENU[2] <= height

    def positionENUControl(self, posENU, yawRadENU):
        self.setPositionControlMode()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        localPosENU = np.array(posENU) - self.localOffsetENU
        self.setpoint.position.x = localPosENU[0]
        self.setpoint.position.y = localPosENU[1]
        self.setpoint.position.z = localPosENU[2]
        self.setpoint.yaw = yawRadENU

    def saturateVelocity(self, velENU):
        return np.clip(velENU, -self.velMax, self.velMax)

    def velocityENUControl(self, velENU, yawRadENU):
        self.setVelocityControlMode()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        velENU = self.saturateVelocity(velENU)
        self.setpoint.velocity.x = velENU[0]
        self.setpoint.velocity.y = velENU[1]
        self.setpoint.velocity.z = velENU[2]
        self.setpoint.yaw = yawRadENU

    def saturateAccleration(self, accENU):
        return np.clip(accENU, -self.accMax, self.accMax)

    def accerlationENUControl(self, accENU, yawRadENU):
        self.setAccelerationControlMode()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        accENU = self.saturateAccleration(accENU)
        self.setpoint.acceleration_or_force.x = accENU[0]
        self.setpoint.acceleration_or_force.y = accENU[1]
        self.setpoint.acceleration_or_force.z = accENU[2]
        self.setpoint.yaw = yawRadENU

    def trajectoryENUControl(self, posENU, velENU, accENU, yawRadENU):
        self.setTrajectoryControlMode()
        self.setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        localPosENU = np.asarray(posENU, dtype=float) - self.localOffsetENU
        velENU = np.asarray(velENU, dtype=float)
        accENU = np.asarray(accENU, dtype=float)

        self.setpoint.position.x = localPosENU[0]
        self.setpoint.position.y = localPosENU[1]
        self.setpoint.position.z = localPosENU[2]
        self.setpoint.velocity.x = velENU[0]
        self.setpoint.velocity.y = velENU[1]
        self.setpoint.velocity.z = velENU[2]
        self.setpoint.acceleration_or_force.x = accENU[0]
        self.setpoint.acceleration_or_force.y = accENU[1]
        self.setpoint.acceleration_or_force.z = accENU[2]
        self.setpoint.yaw = yawRadENU

    def saturateAttitude(self, rpyRad):
        return np.concatenate((np.clip(rpyRad[:2], -self.rpRadMax, self.rpRadMax), rpyRad[2:]))

    def rpyENUThrustControl(self, rpyRadENU, thrust):
        self.setAttitudeControlMode()
        rpyRadENU = self.saturateAttitude(rpyRadENU)
        rpyRadENU[0] += self.rollOffsetRad
        rpyRadENU[1] += self.pitchOffsetRad
        controlQuaternion = euler2quaternion(rpyRadENU)
        self.attitudeSetpoint.thrust = thrust
        self.attitudeSetpoint.orientation.x = controlQuaternion[1]
        self.attitudeSetpoint.orientation.y = controlQuaternion[2]
        self.attitudeSetpoint.orientation.z = controlQuaternion[3]
        self.attitudeSetpoint.orientation.w = controlQuaternion[0]

    def constantVelocityLineENUControl(self, constantVelocityENU, positionENU, yawRadENU):
        kI = 0.05
        kp = 0.5
        self.setVelocityControlMode()
        self.velocityENUControl(constantVelocityENU + kI * (positionENU - self.mePositionENU) + kp * (self.meVelocityENU - constantVelocityENU), yawRadENU)

    def hoverWithYaw(self, yawRadENU):
        self.velocityENUControl([0, 0, 0], yawRadENU)

    def velocityToPointENUControl(self, pointENU, yawRadENU):
        velENU = self.kp * (pointENU - self.mePositionENU)
        self.velocityENUControl(velENU, yawRadENU)
        
    def setGpsOrigin(self, origin: GeoPointStamped = None):
        if origin is None:
            origin = self.gpsOrigin

        origin.header.stamp = rospy.Time.now()

        self.gpsOriginPub.publish(origin)
        rospy.loginfo(f"Setting GPS global origin to: {origin.position.latitude}, {origin.position.longitude}, {origin.position.altitude}")

    def hasSetOrigin(self) -> bool:
        return self.gpsOrigin.position.latitude == self.meGpsOrigin.position.latitude
