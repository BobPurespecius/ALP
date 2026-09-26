#!/usr/bin/env python3

import argparse
import builtins
import copy
import datetime
import json
import math
import os
import pickle
import sys
import time
import threading

import numpy as np
import rospy

from collections import deque

utils_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__)))
sys.path.append(utils_path)
sys.path.append(src_path)
print(sys.path)

from models.P230.P230 import P230
from Communicator import Communicator

from Utils import *
from State import State

PLATFORM_DEFAULT_HOVER_ABOVE_PLATFORM = 0.2

def stepEntrance(method):
    def wrapper(self, *args, **kwargs):
        self.stateStartTime = self.getTimeNow()
        self.stateFinished = False
        return method(self, *args, **kwargs)
    return wrapper


class SingleRun:
    def __init__(self, **kwargs):
        self.scriptPath = os.path.dirname(os.path.realpath(__file__))

        self.number = kwargs.get('number', '')
        self.sceneName = kwargs.get('scene')
        self.plannerBackend = str(kwargs.get('planner_backend') or 'ego').lower()
        self.rvizEnabled = bool(kwargs.get('enable_rviz', False))
        self.takeoff = kwargs.get('takeoff', False)
        
        self.config = json.load(open(os.path.join(utils_path, 'scenes', f'{self.sceneName}.json'), 'rb'))

        self.leadervelocityENU = np.array(self.config['velocityVectorENU'])
        self.formationTime = self.config["formationTime"]
        
        self.obstacleData = self.config.get('obstacleData') or {}
        self.boxObstacleData = self.config.get('boxObstacleData') or {}
        self.movingObstacleData = self.config.get('movingObstacleData') or {}
        self.platformData = self.config.get('platformData') or {}
        self.applyPlatformDefaults()
        self.platformLandOnTop = bool(self.platformData.get('landOnTop', False))
        self.useGazeboTruthForFeedback = bool(self.config.get('useGazeboTruthForFeedback', bool(self.platformData)))
        self.nObstacle = len(self.obstacleData)
        self.nMovingObstacle = len(self.movingObstacleData)
        self.config['nObstacle'] = self.nObstacle
        self.config['nMovingObstacle'] = self.nMovingObstacle

        self.params = json.load(open(os.path.join(utils_path, 'params.json'), 'rb'))
        self.throttleTestOn = self.params['throttle_test']['on']
        self.throttleTestHeight = self.params['throttle_test']['height']
        self.throttleTestChangeTime = self.params['throttle_test']['change_time']
        self.throttleTestMinSim = self.params['throttle_test']['min_guess_sim']
        self.throttleTestMaxSim = self.params['throttle_test']['max_guess_sim']
        self.throttleTestMinReal = self.params['throttle_test']['min_guess_real']
        self.throttleTestMaxReal = self.params['throttle_test']['max_guess_real']
        self.throttleTestAccuracy = self.params['throttle_test']['accuracy']

        self.yawDegENU = self.params['yaw_deg_enu']
        self.yawRadENU = np.deg2rad(self.yawDegENU)
        self.safetyDistanceBetween = self.params['safety']['distance_between']
        self.safetyMaxHeight = self.params['safety']['max_height']
        self.safetyMinHeight = self.params['safety']['min_height']
        self.safetyXMin = self.params['safety']['x_min']
        self.safetyXMax = self.params['safety']['x_max']
        self.safetyYMin = self.params['safety']['y_min']
        self.safetyYMax = self.params['safety']['y_max']

        sceneSafety = self.config.get('safety') or {}
        safetyOverrides = {
            'safetyDistanceBetween': ('distanceBetween', 'distance_between'),
            'safetyMaxHeight': ('maxHeight', 'max_height'),
            'safetyMinHeight': ('minHeight', 'min_height'),
            'safetyXMin': ('xMin', 'x_min'),
            'safetyXMax': ('xMax', 'x_max'),
            'safetyYMin': ('yMin', 'y_min'),
            'safetyYMax': ('yMax', 'y_max'),
        }
        for attribute, keys in safetyOverrides.items():
            for key in keys:
                if key in sceneSafety:
                    setattr(self, attribute, float(sceneSafety[key]))
                    break

        self.tStep = self.params['execution']['time_step']

        timeStr = kwargs.get('prefix') or datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

        self.folderName = os.path.join(self.scriptPath, '..', '..', '..', 'data', 'multi_uav_formation', 'SingleRun', timeStr)
        os.makedirs(self.folderName, exist_ok=True)

        cliOutputFile = open(os.path.join(self.folderName, f'output_{self.number}.txt'), "w")

        originalPrint = print
        lastCliFlushTime = {'t': time.time()}
        def custom_print(*args, **kwargs):
            message = " ".join(map(str, args))
            originalPrint(message, **kwargs)
            cliOutputFile.write(message + "\n")
            now = time.time()
            if now - lastCliFlushTime['t'] >= 0.5:
                cliOutputFile.flush()
                lastCliFlushTime['t'] = now

        builtins.print = custom_print
        self.elapsedTime = self.tStep
        self.tUpperLimit = 100
        self.t = 0
        self.u = np.zeros((3,))
        self.data = []
        self.logLock = threading.Lock()
        self.logSaved = False
        self.lastElapsedPrintTime = 0.0
        self.gazeboPositionENU = None
        self.gazeboVelocityENU = None
        self.gazeboQuaternionENU = None
        self.gazeboPoseTime = None
        self.gazeboOdomTime = None
        self.gazeboOdomState = None
        self.gazeboAppliedOdomState = None
        self.endControlFlag = 0
        self.message = ''

        self.state = State.INIT
        self.stateFinished = False
        self.taskStartTime = time.time()
        self.stateStartTime = time.time()
        self.taskTime = 0
        self.stateTime = 0
        
        self.dragCompensationENU = np.zeros(3)

        self.takeoffPointENU = np.array(self.config["takeoffPointENU"][self.number - 1])
        self.preparePointENU = np.array(self.config["preparePointENU"][self.number - 1])        
        self.takeoffVerticalPointENU = None
        self.normalizePlatformGoals()
        platformReferenceZ = self.platformHoverReferenceZ()
        if self.platformData and platformReferenceZ is not None:
            configuredSafetyMinHeight = self.platformData.get('safetyMinHeight')
            if configuredSafetyMinHeight is not None:
                self.safetyMinHeight = float(configuredSafetyMinHeight)
            safetyHeightMargin = float(self.platformData.get('safetyHeightMargin', 0.8))
            configuredSafetyMaxHeight = self.platformData.get('safetyMaxHeight')
            if configuredSafetyMaxHeight is not None:
                self.safetyMaxHeight = float(configuredSafetyMaxHeight)
            else:
                self.safetyMaxHeight = max(
                    float(platformReferenceZ) + safetyHeightMargin,
                    float(self.takeoffPointENU[2]) + 0.5,
                    float(self.safetyMinHeight) + 0.8
                )
            self.params['safety']['min_height'] = self.safetyMinHeight
            self.params['safety']['max_height'] = self.safetyMaxHeight
        self.egoPlannerConfig = self.config.get('egoPlanner', {})
        self.egoPlannerEnabled = bool(self.egoPlannerConfig.get('enabled', False))
        self.egoObstacleClearance = float(self.egoPlannerConfig.get('obstacleClearance', 0.35))
        self.egoObstacleRepulsionGain = float(self.egoPlannerConfig.get('obstacleRepulsionGain', 0.6))
        self.egoObstacleCheckHorizon = float(self.egoPlannerConfig.get('obstacleCheckHorizon', 1.8))
        self.egoObstacleCheckDt = float(self.egoPlannerConfig.get('obstacleCheckDt', 0.25))
        self.egoObstacleVerticalMargin = float(self.egoPlannerConfig.get('obstacleVerticalMargin', 0.2))
        self.egoObstacleBlockDescentWhenUnsafe = bool(self.egoPlannerConfig.get('obstacleBlockDescentWhenUnsafe', True))
        self.egoObstacleCandidateSpeedScales = list(self.egoPlannerConfig.get(
            'obstacleCandidateSpeedScales',
            [0.0, 0.35, 0.6, 0.85, 1.0]
        ))
        self.egoObstacleCandidateAnglesDeg = list(self.egoPlannerConfig.get(
            'obstacleCandidateAnglesDeg',
            [-120, -90, -60, -35, -20, 0, 20, 35, 60, 90, 120, 180]
        ))
        self.egoObstacleGoalBias = float(self.egoPlannerConfig.get('obstacleGoalBias', 0.45))
        self.egoObstacleStopPenalty = float(self.egoPlannerConfig.get('obstacleStopPenalty', 0.12))
        self.egoMovingObstacleClearanceExtra = float(self.egoPlannerConfig.get('movingObstacleClearanceExtra', 0.25))
        self.egoMovingObstacleTimeBuffer = float(self.egoPlannerConfig.get('movingObstacleTimeBuffer', 0.8))
        self.egoMovingObstacleClosingPenalty = float(self.egoPlannerConfig.get('movingObstacleClosingPenalty', 1.5))
        self.egoMovingObstacleAwayBias = float(self.egoPlannerConfig.get('movingObstacleAwayBias', 0.25))
        self.egoMovingObstacleTimeAwareCostEnabled = bool(self.egoPlannerConfig.get(
            'movingObstacleTimeAwareCostEnabled',
            False
        ))
        configuredMovingObstaclePrediction = bool(self.egoPlannerConfig.get(
            'earlyAvoidanceEnabled',
            self.egoPlannerConfig.get('movingObstaclePredictionEnabled', False)
        ))
        if self.egoMovingObstacleTimeAwareCostEnabled and configuredMovingObstaclePrediction:
            print(
                'Both earlyAvoidanceEnabled and movingObstacleTimeAwareCostEnabled are true; '
                'using README time-aware cost/gradient mode and disabling future point-cloud prediction.'
            )
            configuredMovingObstaclePrediction = False
        self.egoMovingObstaclePredictionEnabled = configuredMovingObstaclePrediction
        if self.egoMovingObstacleTimeAwareCostEnabled:
            self.egoMovingObstacleAvoidanceMode = 'time_aware_cost_gradient'
        elif self.egoMovingObstaclePredictionEnabled:
            self.egoMovingObstacleAvoidanceMode = 'point_cloud_augmentation'
        else:
            self.egoMovingObstacleAvoidanceMode = 'no_early_avoidance'
        self.egoMovingObstaclePointCloudEnabled = bool(self.egoPlannerConfig.get(
            'movingObstaclePointCloudEnabled',
            True
        ))
        self.egoMovingObstaclePredictionHorizon = float(self.egoPlannerConfig.get(
            'movingObstaclePredictionHorizon',
            3.0
        ))
        self.egoMovingObstaclePredictionDt = float(self.egoPlannerConfig.get(
            'movingObstaclePredictionDt',
            0.5
        ))
        self.egoMovingObstaclePredictionRadiusGrowth = float(self.egoPlannerConfig.get(
            'movingObstaclePredictionRadiusGrowth',
            0.04
        ))
        self.egoMovingObstaclePredictionMaxExtraRadius = float(self.egoPlannerConfig.get(
            'movingObstaclePredictionMaxExtraRadius',
            0.18
        ))
        self.egoMovingObstaclePredictionPathMargin = float(self.egoPlannerConfig.get(
            'movingObstaclePredictionPathMargin',
            0.8
        ))
        self.egoMovingObstaclePredictionNearDistance = float(self.egoPlannerConfig.get(
            'movingObstaclePredictionNearDistance',
            2.5
        ))
        self.egoMovingObstaclePredictionMaxCloudsPerObstacle = int(self.egoPlannerConfig.get(
            'movingObstaclePredictionMaxCloudsPerObstacle',
            1
        ))
        defaultExternalAvoidance = False
        self.egoLocalObstacleGuardEnabled = bool(self.egoPlannerConfig.get(
            'localObstacleGuard',
            defaultExternalAvoidance
        ))
        self.egoInterUavRepulsionGain = float(self.egoPlannerConfig.get('interUavRepulsionGain', 1.2))
        self.egoInterUavRecoverySpeed = float(self.egoPlannerConfig.get('interUavRecoverySpeed', 0.55))
        self.egoMaxControlSpeed = float(self.egoPlannerConfig.get('controlVelocityLimit', self.egoPlannerConfig.get('maxVel', 0.9)))
        self.egoObstacleFallbackSpeed = float(self.egoPlannerConfig.get(
            'obstacleFallbackSpeed',
            min(0.45, self.egoMaxControlSpeed)
        ))
        self.egoVelocitySafetyLimit = float(self.egoPlannerConfig.get(
            'velocitySafetyLimit',
            max(1.6, 1.8 * self.egoMaxControlSpeed)
        ))
        self.egoMaxVerticalSpeed = float(self.egoPlannerConfig.get('verticalVelocityLimit', min(0.45, self.egoMaxControlSpeed)))
        self.egoPositionErrorLimit = float(self.egoPlannerConfig.get('positionErrorLimit', 1.5))
        self.egoSafetyMargin = float(self.egoPlannerConfig.get('safetyMargin', 0.35))
        self.egoSafetyZMargin = float(self.egoPlannerConfig.get(
            'safetyZMargin',
            min(0.12, self.egoSafetyMargin)
        ))
        self.egoRecoveryGain = float(self.egoPlannerConfig.get('recoveryGain', 0.8))
        self.egoSafetyRecoveryVerticalLimit = float(self.egoPlannerConfig.get(
            'safetyRecoveryVerticalLimit',
            min(0.45, self.egoMaxVerticalSpeed)
        ))
        self.egoSafetyWarningVerticalLimit = float(self.egoPlannerConfig.get(
            'safetyWarningVerticalLimit',
            min(0.22, self.egoSafetyRecoveryVerticalLimit)
        ))
        self.egoCommandLookahead = float(self.egoPlannerConfig.get('commandLookahead', 1.5))
        self.egoCommandSmoothingEnabled = bool(self.egoPlannerConfig.get('commandSmoothingEnabled', True))
        velocityFilterKey = 'egov2VelocityFilterAlpha' if self.plannerBackend == 'egov2' else 'velocityFilterAlpha'
        commandAccelKey = 'egov2CommandAccelLimit' if self.plannerBackend == 'egov2' else 'commandAccelLimit'
        commandVerticalAccelKey = (
            'egov2CommandVerticalAccelLimit'
            if self.plannerBackend == 'egov2'
            else 'commandVerticalAccelLimit'
        )
        self.egoVelocityFilterAlpha = float(self.egoPlannerConfig.get(
            velocityFilterKey,
            self.egoPlannerConfig.get('velocityFilterAlpha', 0.45)
        ))
        self.egoCommandAccelLimit = float(self.egoPlannerConfig.get(
            commandAccelKey,
            self.egoPlannerConfig.get('commandAccelLimit', 0.8)
        ))
        self.egoCommandVerticalAccelLimit = float(self.egoPlannerConfig.get(
            commandVerticalAccelKey,
            self.egoPlannerConfig.get('commandVerticalAccelLimit', self.egoCommandAccelLimit)
        ))
        self.egoCommandJerkLimit = float(self.egoPlannerConfig.get('commandJerkLimit', 0.0))
        self.lastSmoothedVelocityENU = None
        self.lastSmoothedVelocityTime = None
        self.lastSmoothedAccelENU = None
        self.egoPlannerAvailable = False
        self.egoPlannerTriggered = False
        self.egoGoalReached = False
        self.egoGoalSettledStartTime = None
        self.egoAllFinishedStartTime = None
        self.egoEndDelay = float(self.egoPlannerConfig.get('endDelay', 2.0))
        self.egoLastCommand = None
        self.egoLastCommandTime = None
        self.egoV2VelocityFrameCorrectionENU = np.zeros(3)
        self.egoV2MavrosVelocityCommandENU = np.zeros(3)
        self.egoLastNoCommandFallbackPrintTime = 0.0
        self.egoNoCommandFallbackDelay = float(self.egoPlannerConfig.get('noCommandFallbackDelay', 4.0))
        self.egoOffboardRecoveryPeriod = float(self.egoPlannerConfig.get('offboardRecoveryPeriod', 0.5))
        self.egoFinalConvergenceTime = float(self.egoPlannerConfig.get('finalConvergenceTime', self.formationTime))
        self.egoProgressFallbackTime = float(self.egoPlannerConfig.get('progressFallbackTime', 8.0))
        self.egoProgressFallbackGraceTime = float(self.egoPlannerConfig.get('progressFallbackGraceTime', 12.0))
        self.egoProgressFallbackDistance = float(self.egoPlannerConfig.get('progressFallbackDistance', 2.0))
        self.egoProgressMinImprovement = float(self.egoPlannerConfig.get('progressMinImprovement', 0.5))
        self.egoStaleCommandTime = float(self.egoPlannerConfig.get('staleCommandTime', 4.0))
        self.egoStaleCommandPositionTolerance = float(self.egoPlannerConfig.get('staleCommandPositionTolerance', 0.15))
        self.egoStaleCommandVelocityTolerance = float(self.egoPlannerConfig.get('staleCommandVelocityTolerance', 0.08))
        self.egoStaleCommandGoalDistance = float(self.egoPlannerConfig.get('staleCommandGoalDistance', 1.5))
        self.egoProgressFallbackEnabled = bool(self.egoPlannerConfig.get(
            'progressFallbackEnabled',
            defaultExternalAvoidance
        ))
        self.egoStaleCommandSince = None
        self.egoStaleCommandLastPositionENU = None
        self.egoFinalConvergenceDistance = float(self.egoPlannerConfig.get(
            'finalConvergenceDistance',
            self.egoProgressFallbackDistance
        ))
        self.egoBestGoalDistance = np.inf
        self.egoBestGoalDistanceTime = None
        self.egoFinalConvergenceForced = False
        self.egoLastProgressFallbackPrintTime = 0.0
        self.egoObstacleEscapeEnabled = bool(self.egoPlannerConfig.get(
            'obstacleEscapeEnabled',
            self.egoLocalObstacleGuardEnabled
        ))
        self.egoObstacleEscapeTriggerMargin = float(self.egoPlannerConfig.get('obstacleEscapeTriggerMargin', 0.0))
        self.egoObstacleEscapeVelocityTriggerMargin = float(self.egoPlannerConfig.get(
            'obstacleEscapeVelocityTriggerMargin',
            0.0
        ))
        self.egoObstacleEscapeReleaseMargin = float(self.egoPlannerConfig.get('obstacleEscapeReleaseMargin', 0.25))
        self.egoObstacleEscapeSpeed = float(self.egoPlannerConfig.get(
            'obstacleEscapeSpeed',
            min(0.55, self.egoMaxControlSpeed)
        ))
        self.egoObstacleEscapeVerticalSpeed = float(self.egoPlannerConfig.get(
            'obstacleEscapeVerticalSpeed',
            min(0.35, self.egoMaxVerticalSpeed)
        ))
        self.egoObstacleEscapeAwayGain = float(self.egoPlannerConfig.get('obstacleEscapeAwayGain', 0.6))
        self.egoObstacleEscapeGoalBias = float(self.egoPlannerConfig.get('obstacleEscapeGoalBias', 0.28))
        self.egoObstacleEscapeMinTime = float(self.egoPlannerConfig.get('obstacleEscapeMinTime', 0.8))
        self.egoObstacleEscapeTimeout = float(self.egoPlannerConfig.get('obstacleEscapeTimeout', 8.0))
        self.egoObstacleEscapeCooldown = float(self.egoPlannerConfig.get('obstacleEscapeCooldown', 1.0))
        self.egoObstacleEscapeSwitchMinTime = float(self.egoPlannerConfig.get('obstacleEscapeSwitchMinTime', 1.5))
        self.egoObstacleEscapeSwitchCooldown = float(self.egoPlannerConfig.get('obstacleEscapeSwitchCooldown', 0.8))
        self.egoObstacleEscapeSwitchMargin = float(self.egoPlannerConfig.get('obstacleEscapeSwitchMargin', 0.18))
        self.egoObstacleEscapeRampTime = float(self.egoPlannerConfig.get('obstacleEscapeRampTime', 0.0))
        self.egoObstacleEscapeReleaseBlendTime = float(self.egoPlannerConfig.get('obstacleEscapeReleaseBlendTime', 0.0))
        self.egoObstacleEscapeActive = False
        self.egoObstacleEscapeTargetENU = None
        self.egoObstacleEscapeTangentSign = 1.0
        self.egoObstacleEscapeStartTime = None
        self.egoObstacleEscapeLastSwitchTime = 0.0
        self.egoObstacleEscapeReleaseBlendUntil = 0.0
        self.egoObstacleEscapeObstacleName = ''
        self.egoObstacleEscapeObstacleType = ''
        self.egoObstacleEscapeReason = ''
        self.egoObstacleEscapeEntryMargin = np.inf
        self.egoObstacleEscapeLastReleaseTime = 0.0
        self.egoObstacleEscapeLastPrintTime = 0.0
        self.egoLastObstaclePointMargin = np.inf
        self.egoLastObstacleVelocityMargin = np.inf
        self.egoLastObstacleName = ''
        self.egoLastObstacleType = ''
        self.platformLandingTopZ = self.platformData.get('topZ')
        self.platformLandingTargetZ = self.platformLandingReferenceZ()
        self.platformLandingApproachZ = self.platformHoverReferenceZ()
        self.platformHoverTargetZ = self.platformHoverReferenceZ()
        self.platformFinalHoverActive = False
        self.platformLandingXYTolerance = float(self.platformData.get('landingXYTolerance', 0.6))
        self.platformLandingHeightTolerance = float(self.platformData.get('landingHeightTolerance', 0.35))
        self.platformLandingSpeedTolerance = float(self.platformData.get('landingSpeedTolerance', 0.25))
        self.platformLandingMinimumTime = float(self.platformData.get('landingMinimumTime', 3.0))
        self.platformLandingTimeout = float(self.platformData.get('landingTimeout', 18.0))
        self.platformLandRetryPeriod = float(self.platformData.get('landingRetryPeriod', 2.0))
        self.platformLandingXYGain = float(self.platformData.get('landingXYGain', 0.6))
        self.platformLandingZGain = float(self.platformData.get('landingZGain', 0.8))
        self.platformLandingVelocityDamping = float(self.platformData.get('landingVelocityDamping', 0.45))
        self.platformLandingMaxSpeed = float(self.platformData.get('landingMaxSpeed', 0.45))
        self.platformLandingMaxVerticalSpeed = float(self.platformData.get('landingMaxVerticalSpeed', 0.22))
        self.platformLandingSettleStartTime = None
        self.landCommandSent = False
        self.lastLandCommandTime = 0.0
        self.egoGoalPointENU = None
        self.egoCloudPoints = []
        self.egoLastCloudPointCount = 0
        self.egoLastCloudMovingObstacleCount = 0
        self.egoCurrentCloudMovingObstacleIncludedCount = 0
        self.egoLastCloudMovingObstacleIncludedCount = 0
        self.egoLastCloudPublishWallTime = None
        self.egoLastCloudPredictionEnabled = self.egoMovingObstaclePredictionEnabled
        self.visualizer = None
        self.visualizerPublishPeriod = float(self.egoPlannerConfig.get('obstacleVisualizerPeriod', 0.5))
        self.lastVisualizerPublishTime = 0.0
        self.egoCloudStatusPrintPeriod = float(self.egoPlannerConfig.get('cloudStatusPrintPeriod', 2.0))
        self.lastEgoCloudStatusPrintTime = 0.0
        self.lastOffboardRecoveryAttemptTime = 0.0
        self.statusPrintPeriod = float(self.egoPlannerConfig.get('statusPrintPeriod', 1.0))
        self.lastStatusPrintTime = 0.0
        self.lastSafetyHeightLimitPrintTime = 0.0

        localOffsets = self.config.get('mavrosLocalOffsetENU', [])
        localOffsetENU = localOffsets[self.number - 1] if len(localOffsets) >= self.number else [0.0, 0.0, 0.0]

        rospy.init_node(f'single_run_{self.number}', anonymous=True)
        self.me = P230(name=f'uav{self.number}', localOffsetENU=localOffsetENU)
        self.setupGazeboTruthSubscriber()
        self.communicator = Communicator(self)
        self.setupEgoPlannerBridge()
        self.setupSaveLogService()
        self.spinThread = threading.Thread(target=lambda: rospy.spin())
        self.spinThread.start()

        if self.rvizEnabled and not self.isLeader() and self.hasObstacles():
            from Visualizer import Visualizer
            self.visualizer = Visualizer()
            time.sleep(1)
            self.publishObstacleVisualization(force=True)

        self.timestamps = deque()
        self.hz = int(1 / self.tStep)

        if self.me.mode == 'sim':
            self.throttleTestMin = self.throttleTestMinSim
            self.throttleTestMax = self.throttleTestMaxSim
        elif self.me.mode == 'real':
            self.throttleTestMin = self.throttleTestMinReal
            self.throttleTestMax = self.throttleTestMaxReal
        else:
            raise Exception('Invalid mode')

    def addMessage(self, msg):
        self.message += msg + '\n'

    def goalsCenterENU(self):
        goals = (self.config.get('egoPlanner') or {}).get('goalsENU') or []
        xyGoals = [
            [float(goal[0]), float(goal[1])]
            for goal in goals
            if len(goal) >= 2
        ]
        if not xyGoals:
            return None
        return np.mean(np.array(xyGoals, dtype=float), axis=0).tolist()

    def applyPlatformDefaults(self):
        if not self.platformData:
            return
        if self.platformData.get('centerENU') is None:
            centerENU = self.goalsCenterENU()
            if centerENU is not None:
                self.platformData['centerENU'] = centerENU
                self.config['platformData'] = self.platformData

    def platformVehicleReferenceHeight(self):
        for key in ('vehicleReferenceHeight', 'mavrosReferenceHeight', 'landingReferenceHeight'):
            if key in self.platformData:
                return float(self.platformData[key])
        if hasattr(self, 'takeoffPointENU') and len(self.takeoffPointENU) >= 3:
            return float(self.takeoffPointENU[2])
        return 0.0

    def platformTopReferenceZ(self):
        topZ = self.platformData.get('topZ')
        if topZ is not None:
            return float(topZ)

        size = self.platformData.get('sizeENU')
        if size is not None and len(size) >= 3:
            return float(size[2])

        return None

    def platformLandingReferenceZ(self):
        for key in ('landingTargetZ', 'targetZ'):
            if self.platformData.get(key) is not None:
                return float(self.platformData[key])

        topZ = self.platformTopReferenceZ()
        if topZ is None:
            return None
        return topZ + self.platformVehicleReferenceHeight()

    def configuredGoalReferenceZ(self):
        goals = (self.config.get('egoPlanner') or {}).get('goalsENU') or []
        if len(goals) >= self.number and len(goals[self.number - 1]) >= 3:
            return float(goals[self.number - 1][2])
        return None

    def configuredGoalPointENU(self):
        goals = (self.config.get('egoPlanner') or {}).get('goalsENU') or []
        if len(goals) < self.number:
            return None

        goal = goals[self.number - 1]
        if len(goal) >= 3:
            return np.array([
                float(goal[0]),
                float(goal[1]),
                float(goal[2]),
            ], dtype=float)

        if len(goal) >= 2:
            defaultZ = self.platformDefaultHoverReferenceZ()
            if defaultZ is None:
                return None
            return np.array([
                float(goal[0]),
                float(goal[1]),
                float(defaultZ),
            ], dtype=float)

        return None

    def platformDefaultHoverReferenceZ(self):
        topZ = self.platformTopReferenceZ()
        if topZ is None:
            return None

        hoverAbovePlatform = float(self.platformData.get(
            'hoverAbovePlatform',
            PLATFORM_DEFAULT_HOVER_ABOVE_PLATFORM
        ))
        return topZ + hoverAbovePlatform

    def platformHoverReferenceZ(self):
        goalZ = self.configuredGoalReferenceZ()
        if goalZ is not None:
            return goalZ
        return self.platformDefaultHoverReferenceZ()

    def normalizePlatformGoals(self):
        defaultGoalZ = self.platformDefaultHoverReferenceZ()
        if defaultGoalZ is None:
            return

        goalZ = float(defaultGoalZ)
        if goalZ is None:
            return

        ego = self.config.get('egoPlanner') or {}
        goals = ego.get('goalsENU') or []
        normalizedGoals = []
        changed = False

        for goal in goals:
            if len(goal) >= 2:
                if len(goal) >= 3:
                    normalizedGoals.append([goal[0], goal[1], float(goal[2])])
                else:
                    normalizedGoals.append([goal[0], goal[1], goalZ])
                    changed = True
            else:
                normalizedGoals.append(goal)

        if changed:
            ego['goalsENU'] = normalizedGoals
            self.config['egoPlanner'] = ego

    def setupEgoPlannerBridge(self):
        if not self.egoPlannerEnabled:
            return

        try:
            from std_msgs.msg import Header
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
            from sensor_msgs import point_cloud2
            from sensor_msgs.msg import PointCloud2
            from quadrotor_msgs.msg import PositionCommand
        except ImportError as exc:
            msg = f'EGO planner bridge disabled: {exc}'
            print(msg)
            self.addMessage(msg)
            return

        droneId = self.number - 1
        self.egoHeaderType = Header
        self.egoPoseStampedType = PoseStamped
        self.egoOdometryType = Odometry
        self.egoPointCloud2Type = PointCloud2
        self.egoPointCloud2 = point_cloud2
        self.egoPositionCommandType = PositionCommand
        self.egoCommandTimeout = float(self.egoPlannerConfig.get('commandTimeout', 1.0))
        self.egoCloudPublishPeriod = float(self.egoPlannerConfig.get('cloudPublishPeriod', 0.5))
        self.egoLastCloudPublishTime = 0.0
        self.egoPositionGain = float(self.egoPlannerConfig.get('positionGain', 0.8))
        self.egoVelocityGain = float(self.egoPlannerConfig.get('velocityGain', 0.1))
        self.egoGoalTolerance = float(self.egoPlannerConfig.get('goalTolerance', 0.45))
        self.egoGoalXYTolerance = float(self.egoPlannerConfig.get('goalXYTolerance', self.egoGoalTolerance))
        self.egoGoalZTolerance = float(self.egoPlannerConfig.get('goalZTolerance', 0.12))
        self.egoGoalSpeedTolerance = float(self.egoPlannerConfig.get('goalSpeedTolerance', 0.18))
        self.egoGoalSettleTime = float(self.egoPlannerConfig.get('goalSettleTime', 2.0))
        self.egoHoldXYGain = float(self.egoPlannerConfig.get('holdXYGain', self.egoPositionGain))
        self.egoHoldZGain = float(self.egoPlannerConfig.get('holdZGain', 1.4))
        self.egoHoldVelocityGain = float(self.egoPlannerConfig.get('holdVelocityGain', 0.8))
        self.egoHoldVelocityLimit = float(self.egoPlannerConfig.get('holdVelocityLimit', 0.8))
        self.egoHoldNearDistance = float(self.egoPlannerConfig.get('holdNearDistance', 0.6))
        self.egoHoldNearVelocityLimit = float(self.egoPlannerConfig.get('holdNearVelocityLimit', min(0.35, self.egoHoldVelocityLimit)))
        self.egoHoldNearVerticalVelocityLimit = float(self.egoPlannerConfig.get('holdNearVerticalVelocityLimit', min(0.18, self.egoMaxVerticalSpeed)))
        if self.platformHoverReferenceZ() is not None and not self.platformLandOnTop:
            hoverVerticalLimit = float(self.platformData.get('hoverVerticalVelocityLimit', 0.28))
            self.egoHoldNearVerticalVelocityLimit = max(
                self.egoHoldNearVerticalVelocityLimit,
                min(hoverVerticalLimit, self.egoMaxVerticalSpeed)
            )
        self.egoTriggerDelay = float(self.egoPlannerConfig.get('triggerDelay', 1.0))
        self.egoObstacleClearance = float(self.egoPlannerConfig.get('obstacleClearance', 0.35))
        self.egoObstacleRepulsionGain = float(self.egoPlannerConfig.get('obstacleRepulsionGain', 0.6))
        self.egoLocalObstacleGuardEnabled = False
        self.egoObstacleEscapeGoalBias = float(self.egoPlannerConfig.get('obstacleEscapeGoalBias', 0.18))

        configuredGoalPointENU = self.configuredGoalPointENU()
        if configuredGoalPointENU is not None:
            self.egoGoalPointENU = configuredGoalPointENU

        self.egoOdomPub = rospy.Publisher(
            f'/drone_{droneId}_visual_slam/odom',
            Odometry,
            queue_size=10
        )
        self.egoCloudPub = rospy.Publisher(
            f'/drone_{droneId}_pcl_render_node/cloud',
            PointCloud2,
            queue_size=1,
            latch=True
        )
        self.egoTriggerPub = rospy.Publisher(
            '/traj_start_trigger',
            PoseStamped,
            queue_size=1,
            latch=True
        )
        self.egoCommandSub = rospy.Subscriber(
            f'/drone_{droneId}_planning/pos_cmd',
            PositionCommand,
            self.egoPositionCommandCallback,
            queue_size=10
        )

        self.egoCloudPoints = self.buildEgoObstacleCloud()
        self.egoLastCloudPointCount = len(self.egoCloudPoints)
        self.egoLastCloudMovingObstacleCount = self.nMovingObstacle
        self.egoLastCloudMovingObstacleIncludedCount = self.egoCurrentCloudMovingObstacleIncludedCount
        self.egoLastCloudPredictionEnabled = self.egoMovingObstaclePredictionEnabled
        self.egoPlannerAvailable = True
        self.addMessage(
            f'EGO planner bridge enabled for drone_{droneId}; '
            f'planner_backend={self.plannerBackend}; '
            f'avoidance_mode={self.egoMovingObstacleAvoidanceMode}; '
            f'moving_obstacle_prediction={int(bool(self.egoMovingObstaclePredictionEnabled))}; '
            f'moving_obstacle_cloud={int(bool(self.egoMovingObstaclePointCloudEnabled))}; '
            f'time_aware_cost={int(bool(self.egoMovingObstacleTimeAwareCostEnabled))}'
        )

    def setupGazeboTruthSubscriber(self):
        try:
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
        except ImportError as exc:
            msg = f'Gazebo truth logging disabled: {exc}'
            print(msg)
            self.addMessage(msg)
            return

        def gazeboPoseCallback(msg):
            gazeboPositionENU = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ], dtype=float)
            gazeboQuaternionENU = np.array([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ], dtype=float)
            now = self.getTimeNow()
            self.gazeboPoseTime = now
            odomState = self.gazeboOdomState
            odomIsFresh = (
                odomState is not None and
                now - odomState[3] <= 0.5
            )
            if odomIsFresh:
                return

            self.gazeboPositionENU = gazeboPositionENU
            self.gazeboQuaternionENU = gazeboQuaternionENU

        def gazeboOdomCallback(msg):
            gazeboPositionENU = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ], dtype=float)
            gazeboVelocityENU = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ], dtype=float)
            gazeboQuaternionENU = np.array([
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ], dtype=float)
            now = self.getTimeNow()
            self.gazeboOdomState = (
                gazeboPositionENU,
                gazeboVelocityENU,
                gazeboQuaternionENU,
                now,
            )
            self.gazeboPositionENU = gazeboPositionENU
            self.gazeboVelocityENU = gazeboVelocityENU
            self.gazeboQuaternionENU = gazeboQuaternionENU
            self.gazeboPoseTime = now
            self.gazeboOdomTime = now

        topic = f'/uav_{self.number}/gazebo_pose'
        self.gazeboPoseSub = rospy.Subscriber(
            topic,
            PoseStamped,
            gazeboPoseCallback,
            queue_size=10
        )
        odomTopic = f'/uav_{self.number}/gazebo_odom'
        self.gazeboOdomSub = rospy.Subscriber(
            odomTopic,
            Odometry,
            gazeboOdomCallback,
            queue_size=10
        )
        feedbackLabel = (
            'truth_velocity_feedback'
            if self.plannerBackend == 'egov2'
            else 'truth_odometry_feedback'
        )
        self.addMessage(
            f'Gazebo truth logging enabled from {topic} and {odomTopic}; '
            f'{feedbackLabel}={int(self.useGazeboTruthForFeedback)}'
        )

    def setupSaveLogService(self):
        try:
            from std_srvs.srv import Trigger, TriggerResponse
        except ImportError as exc:
            msg = f'Save-log service disabled: {exc}'
            print(msg)
            self.addMessage(msg)
            return

        def handleSaveLogRequest(_req):
            self.saveLog()
            return TriggerResponse(
                success=True,
                message=f'Data saved to {self.fileName}'
            )

        def handleStatusRequest(_req):
            xyError = None
            zError = None
            speed = None
            if self.egoPlannerEnabled and self.egoGoalPointENU is not None:
                xyError, zError, speed = self.egoGoalErrorComponents()

            status = {
                't': self.t,
                'taskTime': self.taskTime,
                'state': self.state.name,
                'stateFinished': self.stateFinished,
                'fcuConnected': self.me.isConnected(),
                'mavrosMode': self.me.meState.mode,
                'armed': self.me.isArmed(),
                'logSaved': self.logSaved,
                'egoGoalReached': self.egoGoalReached,
                'platformFinalHoverActive': self.platformFinalHoverActive,
                'egoGoalXYError': xyError,
                'egoGoalZError': zError,
                'egoGoalSpeed': speed,
            }
            plainStatus = (
                f"t={self.t:.3f} "
                f"taskTime={self.taskTime:.3f} "
                f"state={self.state.name} "
                f"stateFinished={int(bool(self.stateFinished))} "
                f"fcuConnected={int(bool(self.me.isConnected()))} "
                f"mavrosMode={self.me.meState.mode or 'NONE'} "
                f"armed={int(bool(self.me.isArmed()))} "
                f"logSaved={int(bool(self.logSaved))} "
                f"egoGoalReached={int(bool(self.egoGoalReached))} "
                f"platformFinalHoverActive={int(bool(self.platformFinalHoverActive))} "
                f"egoGoalXYError={xyError if xyError is not None else 'nan'} "
                f"egoGoalZError={zError if zError is not None else 'nan'} "
                f"egoGoalSpeed={speed if speed is not None else 'nan'}"
            )
            return TriggerResponse(
                success=True,
                message=f"{plainStatus} json={json.dumps(status)}"
            )

        self.saveLogService = rospy.Service(
            f'/single_run_{self.number}/save_log',
            Trigger,
            handleSaveLogRequest
        )
        self.statusService = rospy.Service(
            f'/single_run_{self.number}/status',
            Trigger,
            handleStatusRequest
        )
        print(f'Save-log service ready: /single_run_{self.number}/save_log')

    def applyGazeboTruthStateForFeedback(self):
        odomState = self.gazeboOdomState
        truthStateAvailable = (
            self.useGazeboTruthForFeedback and
            odomState is not None and
            self.getTimeNow() - odomState[3] <= 0.5
        )

        if self.plannerBackend == 'egov2':
            # Keep position/velocity atomic on Gazebo truth while retaining the
            # latest MAVROS attitude. Otherwise asynchronous MAVROS callbacks
            # can overwrite only half of the planner feedback state.
            self.me.setExternalWorldStateEnabled(truthStateAvailable)
            if not truthStateAvailable:
                return

            gazeboPositionENU, gazeboVelocityENU, _gazeboQuaternionENU, _ = odomState
            self.me.applyWorldPositionCorrectionENU(gazeboPositionENU)
            self.me.meVelocityENU = np.array(gazeboVelocityENU, dtype=float)
            self.me.meSpeed = np.linalg.norm(self.me.meVelocityENU)
            self.me.applyRawMavrosAttitudeENU()
            self.gazeboAppliedOdomState = odomState
            return

        self.me.setExternalWorldStateEnabled(truthStateAvailable)
        if not truthStateAvailable:
            return

        gazeboPositionENU, gazeboVelocityENU, gazeboQuaternionENU, _ = odomState
        self.me.applyWorldStateCorrectionENU(
            gazeboPositionENU,
            gazeboVelocityENU,
            gazeboQuaternionENU
        )
        self.gazeboAppliedOdomState = odomState

    def egoPositionCommandCallback(self, msg):
        self.updateEgoStaleCommandTracker(msg)
        self.egoLastCommand = msg
        self.egoLastCommandTime = self.getTimeNow()

    def egoV2MavrosVelocityENU(self, worldVelocityENU):
        worldVelocityENU = np.asarray(worldVelocityENU, dtype=float)
        rawVelocityENU = np.asarray(self.me.rawVelocityENU, dtype=float)
        measuredWorldVelocityENU = np.asarray(self.me.meVelocityENU, dtype=float)
        if not (
            np.all(np.isfinite(worldVelocityENU)) and
            np.all(np.isfinite(rawVelocityENU)) and
            np.all(np.isfinite(measuredWorldVelocityENU))
        ):
            self.egoV2VelocityFrameCorrectionENU = np.zeros(3)
            return worldVelocityENU

        # The position target is translated by a time-varying Gazebo-to-MAVROS
        # offset. Apply the offset derivative to velocity so PX4 receives a
        # self-consistent position/velocity trajectory in its local frame.
        self.egoV2VelocityFrameCorrectionENU = rawVelocityENU - measuredWorldVelocityENU
        return worldVelocityENU + self.egoV2VelocityFrameCorrectionENU

    def updateEgoStaleCommandTracker(self, msg):
        posENU = point2Array(msg.position)
        velENU = point2Array(msg.velocity)
        now = self.getTimeNow()

        if self.egoStaleCommandLastPositionENU is None:
            self.egoStaleCommandLastPositionENU = posENU
            self.egoStaleCommandSince = now
            return

        commandMoved = (
            np.linalg.norm(posENU - self.egoStaleCommandLastPositionENU) >
            self.egoStaleCommandPositionTolerance
        )
        commandMoving = np.linalg.norm(velENU) > self.egoStaleCommandVelocityTolerance
        if commandMoved or commandMoving:
            self.egoStaleCommandLastPositionENU = posENU
            self.egoStaleCommandSince = now

    def egoLastCommandIsStaleFarFromGoal(self):
        if (
            self.egoGoalPointENU is None or
            self.egoLastCommand is None or
            self.egoStaleCommandSince is None
        ):
            return False

        posENU = point2Array(self.egoLastCommand.position)
        commandGoalDistance = float(np.linalg.norm(posENU - self.egoGoalPointENU))
        if commandGoalDistance <= self.egoStaleCommandGoalDistance:
            return False

        return self.getTimeNow() - self.egoStaleCommandSince >= self.egoStaleCommandTime

    def hasObstacles(self):
        return bool(
            self.obstacleData or
            self.boxObstacleData or
            self.movingObstacleData or
            self.platformData
        )

    def obstacleTime(self):
        try:
            now = rospy.Time.now().to_sec()
            if now > 0.0:
                return now
        except Exception:
            pass
        return self.t

    def sortedObstacleItems(self, obstacleData):
        def sortKey(item):
            key, _obstacle = item
            try:
                return int(key)
            except ValueError:
                return key

        return sorted(obstacleData.items(), key=sortKey)

    def movingObstacleCenterENU(self, obstacle, t=None):
        if t is None:
            t = self.obstacleTime()

        center = obstacle.get('centerENU', [0.0, 0.0])
        axis = obstacle.get('axisENU', [1.0, 0.0])
        amplitude = float(obstacle.get('amplitude', 0.0))
        period = max(float(obstacle.get('period', 1.0)), 1e-6)
        phase = float(obstacle.get('phase', 0.0))

        if len(axis) < 2:
            unitAxis = np.array([1.0, 0.0])
        else:
            unitAxis = np.array([float(axis[0]), float(axis[1])], dtype=float)
            axisNorm = np.linalg.norm(unitAxis)
            if axisNorm < 1e-6:
                unitAxis = np.array([1.0, 0.0])
            else:
                unitAxis = unitAxis / axisNorm

        offset = amplitude * math.sin(2.0 * math.pi * t / period + phase)
        return [float(center[0]) + unitAxis[0] * offset, float(center[1]) + unitAxis[1] * offset]

    def movingObstacleVelocityENU(self, obstacle, t=None):
        if t is None:
            t = self.obstacleTime()

        axis = obstacle.get('axisENU', [1.0, 0.0])
        amplitude = float(obstacle.get('amplitude', 0.0))
        period = max(float(obstacle.get('period', 1.0)), 1e-6)
        phase = float(obstacle.get('phase', 0.0))

        if len(axis) < 2:
            unitAxis = np.array([1.0, 0.0])
        else:
            unitAxis = np.array([float(axis[0]), float(axis[1])], dtype=float)
            axisNorm = np.linalg.norm(unitAxis)
            if axisNorm < 1e-6:
                unitAxis = np.array([1.0, 0.0])
            else:
                unitAxis = unitAxis / axisNorm

        omega = 2.0 * math.pi / period
        speed = amplitude * omega * math.cos(omega * t + phase)
        return [float(unitAxis[0] * speed), float(unitAxis[1] * speed), 0.0]

    def currentObstacleStates(self, t=None):
        obstacles = []
        for name, obstacle in self.sortedObstacleItems(self.obstacleData):
            current = copy.deepcopy(obstacle)
            current['name'] = name
            current['type'] = 'static'
            current['velocityENU'] = [0.0, 0.0, 0.0]
            obstacles.append(current)

        for name, obstacle in self.sortedObstacleItems(self.movingObstacleData):
            current = copy.deepcopy(obstacle)
            current['name'] = name
            current['type'] = 'moving'
            current['centerENU'] = self.movingObstacleCenterENU(obstacle, t=t)
            current['velocityENU'] = self.movingObstacleVelocityENU(obstacle, t=t)
            obstacles.append(current)

        return obstacles

    def buildEgoObstacleCloud(self):
        if not self.hasObstacles():
            return []

        resolution = float(self.egoPlannerConfig.get('cloudResolution', 0.2))
        defaultHeight = float(self.egoPlannerConfig.get('obstacleHeight', 3.0))
        points = []
        includedMovingObstacleCount = 0

        def appendObstacleCloud(obstacle, radiusExtra=0.0):
            center = obstacle.get('centerENU', [0.0, 0.0])
            radius = float(obstacle.get('radius', 0.5)) + max(float(radiusExtra), 0.0)
            height = float(obstacle.get('height', defaultHeight))
            zMin = float(obstacle.get('zMin', 0.0))
            cx, cy = float(center[0]), float(center[1])

            xs = np.arange(cx - radius, cx + radius + resolution * 0.5, resolution)
            ys = np.arange(cy - radius, cy + radius + resolution * 0.5, resolution)
            zs = np.arange(zMin, zMin + height + resolution * 0.5, resolution)

            for x in xs:
                for y in ys:
                    if (x - cx) ** 2 + (y - cy) ** 2 > radius ** 2:
                        continue
                    for z in zs:
                        points.append([float(x), float(y), float(z)])

        def appendBoxObstacleCloud(obstacle):
            center = obstacle.get('centerENU')
            size = obstacle.get('sizeENU')
            if center is None or len(center) < 2 or size is None or len(size) < 3:
                return

            halfX = 0.5 * float(size[0])
            halfY = 0.5 * float(size[1])
            height = float(size[2])
            if halfX <= 0.0 or halfY <= 0.0 or height <= 0.0:
                return

            centerX, centerY = float(center[0]), float(center[1])
            minX, maxX = centerX - halfX, centerX + halfX
            minY, maxY = centerY - halfY, centerY + halfY
            minZ = float(obstacle.get('zMin', 0.0))
            maxZ = minZ + height

            def samples(start, stop):
                intervals = max(int(math.ceil((stop - start) / resolution)), 1)
                return np.linspace(start, stop, intervals + 1)

            xSamples = samples(minX, maxX)
            ySamples = samples(minY, maxY)
            zSamples = samples(minZ, maxZ)
            for x in xSamples:
                for y in ySamples:
                    points.append([float(x), float(y), minZ])
                    points.append([float(x), float(y), maxZ])
            for x in (minX, maxX):
                for y in ySamples:
                    for z in zSamples:
                        points.append([float(x), float(y), float(z)])
            for y in (minY, maxY):
                for x in xSamples:
                    for z in zSamples:
                        points.append([float(x), float(y), float(z)])

        def appendPlatformCloud():
            if not self.platformData:
                return

            center = self.platformData.get('centerENU')
            size = self.platformData.get('sizeENU')
            topZ = self.platformTopReferenceZ()
            if center is None or len(center) < 2 or size is None or len(size) < 3 or topZ is None:
                return

            halfX = 0.5 * float(size[0])
            halfY = 0.5 * float(size[1])
            height = float(size[2])
            if halfX <= 0.0 or halfY <= 0.0 or height <= 0.0:
                return

            mapResolution = float(self.egoPlannerConfig.get('mapResolution', 0.1))
            obstacleInflation = float(self.egoPlannerConfig.get('obstacleInflation', 0.35))
            inflateSteps = int(math.ceil(max(obstacleInflation, 0.0) / max(mapResolution, 1e-3)))
            verticalInflation = inflateSteps * mapResolution

            centerX, centerY = float(center[0]), float(center[1])
            minX, maxX = centerX - halfX, centerX + halfX
            minY, maxY = centerY - halfY, centerY + halfY
            minZ = float(topZ) - height
            cloudTopZ = max(minZ, float(topZ) - verticalInflation)

            def samples(start, stop):
                intervals = max(int(math.ceil((stop - start) / resolution)), 1)
                return np.linspace(start, stop, intervals + 1)

            xSamples = samples(minX, maxX)
            ySamples = samples(minY, maxY)
            zSamples = samples(minZ, cloudTopZ)

            # The grid inflates these samples back to the physical deck height.
            # This preserves the Gazebo box boundary while keeping hover goals
            # above the deck reachable by both planner backends.
            for x in xSamples:
                for y in ySamples:
                    points.append([float(x), float(y), float(cloudTopZ)])
            for x in (minX, maxX):
                for y in ySamples:
                    for z in zSamples:
                        points.append([float(x), float(y), float(z)])
            for y in (minY, maxY):
                for x in xSamples:
                    for z in zSamples:
                        points.append([float(x), float(y), float(z)])

        def pointSegmentDistanceXY(pointXY, startXY, endXY):
            segmentXY = endXY - startXY
            lengthSq = float(np.dot(segmentXY, segmentXY))
            if lengthSq <= 1e-6:
                return float(np.linalg.norm(pointXY - startXY))
            ratio = float(np.clip(np.dot(pointXY - startXY, segmentXY) / lengthSq, 0.0, 1.0))
            closestXY = startXY + ratio * segmentXY
            return float(np.linalg.norm(pointXY - closestXY))

        def predictionRelevanceScore(obstacle, predictedCenter, tau):
            centerXY = np.array(predictedCenter, dtype=float)[:2]
            currentXY = np.array(self.me.mePositionENU, dtype=float)[:2]
            nearDistance = float(np.linalg.norm(centerXY - currentXY))
            radius = float(obstacle.get('radius', 0.5))
            nearLimit = radius + self.egoMovingObstaclePredictionNearDistance

            if self.egoGoalPointENU is None:
                return nearDistance if nearDistance <= nearLimit else None

            goalXY = np.array(self.egoGoalPointENU, dtype=float)[:2]
            pathDistance = pointSegmentDistanceXY(centerXY, currentXY, goalXY)
            pathLimit = radius + self.egoMovingObstaclePredictionPathMargin
            if pathDistance <= pathLimit or nearDistance <= nearLimit:
                return min(pathDistance, nearDistance) + 0.08 * float(tau)
            return None

        for obstacle in self.currentObstacleStates():
            if obstacle.get('type') == 'moving':
                if not self.egoMovingObstaclePointCloudEnabled:
                    continue
                includedMovingObstacleCount += 1
            appendObstacleCloud(obstacle)

        for _name, obstacle in self.sortedObstacleItems(self.boxObstacleData):
            appendBoxObstacleCloud(obstacle)

        appendPlatformCloud()

        if self.egoMovingObstaclePredictionEnabled and self.movingObstacleData:
            baseTime = self.obstacleTime()
            horizon = max(self.egoMovingObstaclePredictionHorizon, 0.0)
            dt = max(self.egoMovingObstaclePredictionDt, resolution)
            if horizon > 1e-6:
                maxClouds = max(self.egoMovingObstaclePredictionMaxCloudsPerObstacle, 0)
                for _name, obstacle in self.sortedObstacleItems(self.movingObstacleData):
                    candidates = []
                    for tau in np.arange(dt, horizon + 0.5 * dt, dt):
                        predictedCenter = self.movingObstacleCenterENU(
                            obstacle,
                            t=baseTime + float(tau)
                        )
                        score = predictionRelevanceScore(obstacle, predictedCenter, tau)
                        if score is None:
                            continue
                        candidates.append((score, float(tau), predictedCenter))

                    candidates.sort(key=lambda item: item[0])
                    for _score, tau, predictedCenter in candidates[:maxClouds]:
                        radiusExtra = min(
                            self.egoMovingObstaclePredictionRadiusGrowth * float(tau),
                            self.egoMovingObstaclePredictionMaxExtraRadius
                        )
                        predicted = copy.deepcopy(obstacle)
                        predicted['type'] = 'moving_prediction'
                        predicted['centerENU'] = predictedCenter
                        appendObstacleCloud(predicted, radiusExtra=radiusExtra)

        self.egoCurrentCloudMovingObstacleIncludedCount = includedMovingObstacleCount
        return points

    def publishEgoPlannerInputs(self):
        if not (self.egoPlannerEnabled and self.egoPlannerAvailable):
            return

        now = rospy.Time.now()

        odom = self.egoOdometryType()
        odom.header.stamp = now
        odom.header.frame_id = 'world'
        odom.child_frame_id = f'uav{self.number}/base_link'
        odom.pose.pose.position.x = self.me.mePositionENU[0]
        odom.pose.pose.position.y = self.me.mePositionENU[1]
        odom.pose.pose.position.z = self.me.mePositionENU[2]
        odom.pose.pose.orientation.w = self.me.meQuaternionENU[0]
        odom.pose.pose.orientation.x = self.me.meQuaternionENU[1]
        odom.pose.pose.orientation.y = self.me.meQuaternionENU[2]
        odom.pose.pose.orientation.z = self.me.meQuaternionENU[3]
        odom.twist.twist.linear.x = self.me.meVelocityENU[0]
        odom.twist.twist.linear.y = self.me.meVelocityENU[1]
        odom.twist.twist.linear.z = self.me.meVelocityENU[2]
        self.egoOdomPub.publish(odom)

        nowSec = self.getTimeNow()
        if self.hasObstacles() and nowSec - self.egoLastCloudPublishTime >= self.egoCloudPublishPeriod:
            self.egoCloudPoints = self.buildEgoObstacleCloud()
            self.egoLastCloudPointCount = len(self.egoCloudPoints)
            self.egoLastCloudMovingObstacleCount = self.nMovingObstacle
            self.egoLastCloudMovingObstacleIncludedCount = self.egoCurrentCloudMovingObstacleIncludedCount
            self.egoLastCloudPredictionEnabled = self.egoMovingObstaclePredictionEnabled
            if not self.egoCloudPoints:
                self.egoLastCloudPublishTime = nowSec
                if nowSec - self.lastEgoCloudStatusPrintTime >= self.egoCloudStatusPrintPeriod:
                    print(
                        f'EGO obstacle cloud empty: planner={self.plannerBackend}, '
                        f'moving={self.nMovingObstacle}, '
                        f'moving_in_cloud={self.egoLastCloudMovingObstacleIncludedCount}, '
                        f'prediction={int(bool(self.egoMovingObstaclePredictionEnabled))}'
                    )
                    self.lastEgoCloudStatusPrintTime = nowSec
                return
            header = self.egoHeaderType()
            header.stamp = now
            header.frame_id = 'world'
            cloud = self.egoPointCloud2.create_cloud_xyz32(header, self.egoCloudPoints)
            self.egoCloudPub.publish(cloud)
            self.egoLastCloudPublishTime = nowSec
            self.egoLastCloudPublishWallTime = nowSec
            if nowSec - self.lastEgoCloudStatusPrintTime >= self.egoCloudStatusPrintPeriod:
                print(
                    f'EGO obstacle cloud published: planner={self.plannerBackend}, '
                    f'topic=/drone_{self.number - 1}_pcl_render_node/cloud, '
                    f'points={self.egoLastCloudPointCount}, '
                    f'moving={self.egoLastCloudMovingObstacleCount}, '
                    f'moving_in_cloud={self.egoLastCloudMovingObstacleIncludedCount}, '
                    f'prediction={int(bool(self.egoMovingObstaclePredictionEnabled))}'
                )
                self.lastEgoCloudStatusPrintTime = nowSec

    def publishEgoStartTrigger(self):
        if not (self.egoPlannerEnabled and self.egoPlannerAvailable):
            return
        if self.egoPlannerTriggered:
            return
        if self.stateTime < self.egoTriggerDelay:
            return

        trigger = self.egoPoseStampedType()
        trigger.header.stamp = rospy.Time.now()
        trigger.header.frame_id = 'world'
        trigger.pose.position.x = self.me.mePositionENU[0]
        trigger.pose.position.y = self.me.mePositionENU[1]
        trigger.pose.position.z = self.me.mePositionENU[2]
        trigger.pose.orientation.w = 1.0
        self.egoTriggerPub.publish(trigger)
        self.egoPlannerTriggered = True
        self.addMessage('EGO planner start trigger published')

    def publishObstacleVisualization(self, force=False):
        if self.visualizer is None:
            return

        nowSec = self.getTimeNow()
        if not force and nowSec - self.lastVisualizerPublishTime < self.visualizerPublishPeriod:
            return

        defaultHeight = float(self.egoPlannerConfig.get('obstacleHeight', 3.0))
        self.visualizer.set_cylinders(
            self.currentObstacleStates(),
            default_height=defaultHeight,
            platform=self.platformData
        )
        self.visualizer.publish_once()
        self.lastVisualizerPublishTime = nowSec

    def clipVelocityENU(self, velENU, maxSpeed=None, maxVerticalSpeed=None):
        velENU = np.array(velENU, dtype=float)
        if not np.all(np.isfinite(velENU)):
            return np.zeros(3)

        if maxSpeed is None:
            maxSpeed = self.egoMaxControlSpeed
        if maxVerticalSpeed is None:
            maxVerticalSpeed = self.egoMaxVerticalSpeed

        maxSpeed = max(float(maxSpeed), 1e-6)
        maxVerticalSpeed = max(float(maxVerticalSpeed), 1e-6)
        velENU[2] = np.clip(velENU[2], -maxVerticalSpeed, maxVerticalSpeed)

        horizontalNorm = np.linalg.norm(velENU[:2])
        horizontalLimit = math.sqrt(max(maxSpeed ** 2 - velENU[2] ** 2, 0.0))
        if horizontalNorm > horizontalLimit > 1e-6:
            velENU[:2] *= horizontalLimit / horizontalNorm

        norm = np.linalg.norm(velENU)
        if norm > maxSpeed:
            velENU *= maxSpeed / norm

        return velENU

    def clipVectorNorm(self, vector, maxNorm):
        vector = np.array(vector, dtype=float)
        if not np.all(np.isfinite(vector)):
            return np.zeros_like(vector)

        norm = np.linalg.norm(vector)
        if norm > maxNorm > 1e-6:
            return vector * (maxNorm / norm)
        return vector

    def resetSmoothedVelocityCommand(self, initialVelocityENU=None):
        if initialVelocityENU is None:
            initialVelocityENU = np.zeros(3)
        self.lastSmoothedVelocityENU = np.array(initialVelocityENU, dtype=float)
        self.lastSmoothedVelocityTime = self.getTimeNow()
        self.lastSmoothedAccelENU = np.zeros(3, dtype=float)

    def smoothVelocityCommandENU(self, targetVelENU):
        targetVelENU = np.array(targetVelENU, dtype=float)
        if not self.egoCommandSmoothingEnabled:
            return targetVelENU

        alpha = float(np.clip(self.egoVelocityFilterAlpha, 0.0, 1.0))
        accelLimit = max(float(self.egoCommandAccelLimit), 1e-6)
        now = self.getTimeNow()
        if (
            self.egoObstacleEscapeReleaseBlendTime > 1e-3 and
            now < self.egoObstacleEscapeReleaseBlendUntil
        ):
            accelLimit *= 0.5

        if self.lastSmoothedVelocityENU is None or self.lastSmoothedVelocityTime is None:
            self.resetSmoothedVelocityCommand(self.me.meVelocityENU)

        dt = max(now - self.lastSmoothedVelocityTime, self.tStep)
        limitedDt = min(dt, 0.25)
        previousVelENU = np.array(self.lastSmoothedVelocityENU, dtype=float)

        desiredDeltaENU = targetVelENU - previousVelENU
        desiredAccelENU = desiredDeltaENU / max(limitedDt, 1e-6)
        desiredAccelENU[:2] = self.clipVectorNorm(desiredAccelENU[:2], accelLimit)
        verticalAccelLimit = max(float(self.egoCommandVerticalAccelLimit), 1e-6)
        desiredAccelENU[2] = np.clip(
            desiredAccelENU[2],
            -verticalAccelLimit,
            verticalAccelLimit
        )

        if self.lastSmoothedAccelENU is None:
            self.lastSmoothedAccelENU = np.zeros(3, dtype=float)
        previousAccelENU = np.array(self.lastSmoothedAccelENU, dtype=float)
        jerkLimit = max(float(self.egoCommandJerkLimit), 0.0)
        if jerkLimit > 1e-6:
            desiredAccelENU = previousAccelENU + self.clipVectorNorm(
                desiredAccelENU - previousAccelENU,
                jerkLimit * limitedDt
            )

        accelLimitedVelENU = previousVelENU + desiredAccelENU * limitedDt
        smoothedVelENU = (1.0 - alpha) * previousVelENU + alpha * accelLimitedVelENU
        smoothedVelENU = self.clipVelocityENU(smoothedVelENU)
        actualAccelENU = (smoothedVelENU - previousVelENU) / max(limitedDt, 1e-6)

        self.lastSmoothedVelocityENU = smoothedVelENU
        self.lastSmoothedVelocityTime = now
        self.lastSmoothedAccelENU = actualAccelENU
        return smoothedVelENU

    def safetyBoundaryViolation(self, margin=0.0):
        posENU = self.me.mePositionENU
        zMargin = min(float(margin), self.egoSafetyZMargin)
        if posENU[2] > self.safetyMaxHeight - zMargin:
            return 'too high'
        if posENU[2] < self.safetyMinHeight + zMargin:
            return 'too low'
        if posENU[0] < self.safetyXMin + margin:
            return 'x below safety range'
        if posENU[0] > self.safetyXMax - margin:
            return 'x above safety range'
        if posENU[1] < self.safetyYMin + margin:
            return 'y below safety range'
        if posENU[1] > self.safetyYMax - margin:
            return 'y above safety range'
        return ''

    def safetyBoundaryIsHeight(self, reason):
        return reason in ('too high', 'too low')

    def applySafetyHeightLimitToVelocity(self, velENU):
        velENU = np.array(velENU, dtype=float)
        if not np.all(np.isfinite(velENU)):
            return np.zeros(3), ''

        posZ = float(self.me.mePositionENU[2])
        currentVz = float(self.me.meVelocityENU[2])
        softMargin = max(self.egoSafetyMargin, self.egoSafetyZMargin)
        upperSoftZ = self.safetyMaxHeight - softMargin
        lowerSoftZ = self.safetyMinHeight + softMargin
        reason = ''

        if posZ >= upperSoftZ:
            reason = 'too high'
            violation = posZ >= self.safetyMaxHeight
            penetration = max(posZ - upperSoftZ, 0.0)
            softBand = max(self.safetyMaxHeight - upperSoftZ, 1e-3)
            ramp = float(np.clip(penetration / softBand, 0.0, 1.0))
            verticalLimit = (
                self.egoSafetyRecoveryVerticalLimit
                if violation
                else self.egoSafetyWarningVerticalLimit +
                ramp * (self.egoSafetyRecoveryVerticalLimit - self.egoSafetyWarningVerticalLimit)
            )
            desiredVz = -np.clip(
                0.08 + self.egoRecoveryGain * penetration + 0.25 * max(currentVz, 0.0),
                0.08,
                verticalLimit
            )
            velENU[2] = min(velENU[2], desiredVz)
        elif posZ <= lowerSoftZ:
            reason = 'too low'
            violation = posZ <= self.safetyMinHeight
            penetration = max(lowerSoftZ - posZ, 0.0)
            softBand = max(lowerSoftZ - self.safetyMinHeight, 1e-3)
            ramp = float(np.clip(penetration / softBand, 0.0, 1.0))
            verticalLimit = (
                self.egoSafetyRecoveryVerticalLimit
                if violation
                else self.egoSafetyWarningVerticalLimit +
                ramp * (self.egoSafetyRecoveryVerticalLimit - self.egoSafetyWarningVerticalLimit)
            )
            desiredVz = np.clip(
                0.08 + self.egoRecoveryGain * penetration + 0.25 * max(-currentVz, 0.0),
                0.08,
                verticalLimit
            )
            velENU[2] = max(velENU[2], desiredVz)

        if reason:
            now = self.getTimeNow()
            if now - self.lastSafetyHeightLimitPrintTime >= 0.5:
                print(
                    f'Safety height velocity limit ({reason}): '
                    f'z={posZ:.2f}, range=({self.safetyMinHeight:.2f}, {self.safetyMaxHeight:.2f}), '
                    f'cmd={arrayString(velENU)}'
                )
                self.lastSafetyHeightLimitPrintTime = now

        return velENU, reason

    def pointInsideSafetyBox(self, pointENU, margin=0.0):
        pointENU = np.array(pointENU, dtype=float)
        return (
            self.safetyXMin + margin <= pointENU[0] <= self.safetyXMax - margin and
            self.safetyYMin + margin <= pointENU[1] <= self.safetyYMax - margin and
            self.safetyMinHeight + margin <= pointENU[2] <= self.safetyMaxHeight - margin
        )

    def sendBoundedVelocityENUControl(self, velENU, yawRadENU, useObstacleGuard=False, smooth=True):
        boundedVelENU = self.clipVelocityENU(velENU)
        if useObstacleGuard and self.egoLocalObstacleGuardEnabled:
            boundedVelENU = self.projectVelocityAwayFromObstacles(boundedVelENU)

        if smooth:
            boundedVelENU = self.smoothVelocityCommandENU(boundedVelENU)

        boundedVelENU, heightLimitReason = self.applySafetyHeightLimitToVelocity(boundedVelENU)
        boundedVelENU = self.clipVelocityENU(boundedVelENU)
        if heightLimitReason and self.lastSmoothedVelocityENU is not None:
            self.lastSmoothedVelocityENU = np.array(boundedVelENU, dtype=float)
            self.lastSmoothedVelocityTime = self.getTimeNow()
            self.lastSmoothedAccelENU = np.zeros(3, dtype=float)
        elif not smooth:
            self.resetSmoothedVelocityCommand(boundedVelENU)

        self.u = boundedVelENU
        self.me.velocityENUControl(boundedVelENU, yawRadENU)
        return boundedVelENU

    def safetyRecoveryVelocity(self, reason=None):
        margin = self.egoSafetyMargin
        posENU = self.me.mePositionENU
        boundaryViolation = self.safetyBoundaryViolation(margin=0.0)
        if boundaryViolation == 'too low' or reason == 'too low':
            velENU = np.zeros(3, dtype=float)
            verticalLimit = (
                self.egoSafetyRecoveryVerticalLimit
                if boundaryViolation == 'too low'
                else self.egoSafetyWarningVerticalLimit
            )
            velENU[2] = np.clip(
                self.egoRecoveryGain * (self.safetyMinHeight + margin - posENU[2]) -
                0.25 * self.me.meVelocityENU[2],
                0.0,
                verticalLimit
            )
            return self.clipVelocityENU(
                velENU,
                maxVerticalSpeed=verticalLimit
            )
        if boundaryViolation == 'too high' or reason == 'too high':
            velENU = np.zeros(3, dtype=float)
            verticalLimit = (
                self.egoSafetyRecoveryVerticalLimit
                if boundaryViolation == 'too high'
                else self.egoSafetyWarningVerticalLimit
            )
            velENU[2] = np.clip(
                self.egoRecoveryGain * (self.safetyMaxHeight - margin - posENU[2]) -
                0.25 * self.me.meVelocityENU[2],
                -verticalLimit,
                0.0
            )
            return self.clipVelocityENU(
                velENU,
                maxVerticalSpeed=verticalLimit
            )

        targetENU = np.array([
            np.clip(posENU[0], self.safetyXMin + margin, self.safetyXMax - margin),
            np.clip(posENU[1], self.safetyYMin + margin, self.safetyYMax - margin),
            np.clip(posENU[2], self.safetyMinHeight + margin, self.safetyMaxHeight - margin),
        ], dtype=float)

        if not boundaryViolation:
            if self.egoGoalPointENU is not None:
                targetENU = np.array(self.egoGoalPointENU, dtype=float)
            else:
                targetENU = np.array(self.preparePointENU, dtype=float)
            targetENU[0] = np.clip(targetENU[0], self.safetyXMin + margin, self.safetyXMax - margin)
            targetENU[1] = np.clip(targetENU[1], self.safetyYMin + margin, self.safetyYMax - margin)
            targetENU[2] = np.clip(targetENU[2], self.safetyMinHeight + margin, self.safetyMaxHeight - margin)

        velENU = self.egoRecoveryGain * (targetENU - posENU) - 0.4 * self.me.meVelocityENU
        return self.clipVelocityENU(velENU)

    def interUavSeparationVelocityENU(self):
        separationVelENU = np.zeros(3)
        closestDistance = np.inf
        closestUav = None

        for uavNumber, info in self.communicator.othersInfo.items():
            otherPositionENU = point2Array(info['position'])
            if not np.all(np.isfinite(otherPositionENU)):
                continue

            deltaENU = self.me.mePositionENU - otherPositionENU
            deltaENU[2] = 0.0
            distance = float(np.linalg.norm(deltaENU))
            if distance < closestDistance:
                closestDistance = distance
                closestUav = uavNumber
            if distance >= self.safetyDistanceBetween:
                continue

            if distance < 1e-3:
                directionENU = np.array([
                    0.0,
                    1.0 if self.number >= int(uavNumber) else -1.0,
                    0.0
                ])
            else:
                directionENU = deltaENU / distance

            penetration = self.safetyDistanceBetween - distance
            separationVelENU += self.egoInterUavRepulsionGain * max(penetration, 0.08) * directionENU

        if np.linalg.norm(separationVelENU[:2]) <= 1e-6:
            return separationVelENU, closestUav, closestDistance

        separationVelENU[:2] = self.clipVectorNorm(
            separationVelENU[:2],
            self.egoInterUavRecoverySpeed
        )
        return separationVelENU, closestUav, closestDistance

    def recoverFromGuidanceSafety(self):
        separationVelENU, closestUav, closestDistance = self.interUavSeparationVelocityENU()
        if np.linalg.norm(separationVelENU[:2]) <= 1e-6:
            self.recoverFromSafetyBoundary('safety module blocked EGO guidance')
            return

        if self.egoGoalPointENU is not None:
            goalErrorENU = self.egoGoalPointENU - self.me.mePositionENU
            separationVelENU[2] = np.clip(
                self.egoHoldZGain * goalErrorENU[2],
                -self.egoMaxVerticalSpeed,
                self.egoMaxVerticalSpeed
            )

        recoveryVelENU = self.sendBoundedVelocityENUControl(
            separationVelENU,
            self.yawRadENU,
            useObstacleGuard=False,
            smooth=False
        )
        print(
            f'Inter-UAV safety recovery active: closest={closestUav}, '
            f'distance={closestDistance:.2f}, vel cmd={arrayString(recoveryVelENU)}'
        )

    def recoverFromSafetyBoundary(self, reason):
        recoveryVelENU = self.safetyRecoveryVelocity(reason=reason)
        recoveryVelENU = self.sendBoundedVelocityENUControl(
            recoveryVelENU,
            self.yawRadENU,
            useObstacleGuard=False,
            smooth=False
        )
        print(f'Safety recovery active ({reason}), vel cmd = {arrayString(recoveryVelENU)}')

    def guidanceHardSafetyViolation(self):
        if self.plannerBackend != 'egov2':
            for uav_name, info in self.communicator.othersInfo.items():
                if self.me.nearPositionENU(point2Array(info['position']), tol=self.safetyDistanceBetween):
                    print(f'Safety module: too close to {uav_name}, quit...')
                    return f'too close to {uav_name}'
        if self.me.mePositionENU[0] < self.safetyXMin or self.me.mePositionENU[0] > self.safetyXMax:
            print(f'Safety module: x ({self.me.mePositionENU[0]:.2f}) is out of range ({self.safetyXMin:.2f}, {self.safetyXMax:.2f}), quit...')
            return 'x outside safety range'
        if self.me.mePositionENU[1] < self.safetyYMin or self.me.mePositionENU[1] > self.safetyYMax:
            print(f'Safety module: y ({self.me.mePositionENU[1]:.2f}) is out of range ({self.safetyYMin:.2f}, {self.safetyYMax:.2f}), quit...')
            return 'y outside safety range'
        return ''

    def egoCommandIsUsable(self, posENU, velENU):
        if not (np.all(np.isfinite(posENU)) and np.all(np.isfinite(velENU))):
            print('Rejected EGO command: non-finite position or velocity')
            return False

        commandOutsideSafetyBox = (
            posENU[0] < self.safetyXMin or posENU[0] > self.safetyXMax or
            posENU[1] < self.safetyYMin or posENU[1] > self.safetyYMax
        )
        if self.plannerBackend == 'egov2':
            commandOutsideSafetyBox = commandOutsideSafetyBox or (
                posENU[2] < self.safetyMinHeight or posENU[2] > self.safetyMaxHeight
            )
        if commandOutsideSafetyBox:
            print(f'Rejected EGO command outside safety box: target = {arrayString(posENU)}')
            return False

        predictedENU = self.me.mePositionENU + self.clipVelocityENU(velENU) * self.egoCommandLookahead
        predictedBoundaryViolation = (
            predictedENU[0] < self.safetyXMin or predictedENU[0] > self.safetyXMax or
            predictedENU[1] < self.safetyYMin or predictedENU[1] > self.safetyYMax
        )
        if self.plannerBackend == 'egov2':
            predictedBoundaryViolation = predictedBoundaryViolation or (
                predictedENU[2] < self.safetyMinHeight or predictedENU[2] > self.safetyMaxHeight
            )
        if predictedBoundaryViolation:
            print(f'Rejected EGO command: predicted boundary violation at {arrayString(predictedENU)}')
            return False

        return True

    def egoCommandVelocity(self, posENU, velENU):
        posENU = np.array(posENU, dtype=float)
        velENU = np.array(velENU, dtype=float)
        clippedPosENU = np.array(posENU, dtype=float)
        clippedPosENU[2] = np.clip(
            clippedPosENU[2],
            self.safetyMinHeight + self.egoSafetyZMargin,
            self.safetyMaxHeight - self.egoSafetyZMargin
        )
        if abs(clippedPosENU[2] - posENU[2]) > 1e-3:
            print(
                f'Clipped EGO command z from {posENU[2]:.2f} to {clippedPosENU[2]:.2f} '
                f'within safety height'
            )
        posErrorENU = self.clipVectorNorm(
            clippedPosENU - self.me.mePositionENU,
            self.egoPositionErrorLimit
        )
        velErrorENU = velENU - self.me.meVelocityENU
        controlVelENU = velENU + self.egoPositionGain * posErrorENU + self.egoVelocityGain * velErrorENU
        return self.clipVelocityENU(controlVelENU)

    def obstacleVerticalRange(self, obstacle):
        defaultHeight = float(self.egoPlannerConfig.get('obstacleHeight', 3.0))
        zMin = float(obstacle.get('zMin', 0.0))
        height = float(obstacle.get('height', defaultHeight))
        return zMin, zMin + height

    def obstacleSafeRadius(self, obstacle, posXY=None, velXY=None):
        radius = float(obstacle.get('radius', 0.5))
        safeRadius = radius + self.egoObstacleClearance

        if obstacle.get('type') != 'moving':
            return safeRadius

        safeRadius += self.egoMovingObstacleClearanceExtra
        if posXY is None or velXY is None:
            return safeRadius

        center = np.array(obstacle.get('centerENU', [0.0, 0.0]), dtype=float)[:2]
        obstacleVelXY = np.array(
            obstacle.get('velocityENU', [0.0, 0.0, 0.0]),
            dtype=float
        )[:2]
        offsetXY = np.array(posXY, dtype=float)[:2] - center
        offsetNorm = float(np.linalg.norm(offsetXY))
        if offsetNorm > 1e-4:
            offsetUnit = offsetXY / offsetNorm
            relativeVelXY = np.array(velXY, dtype=float)[:2] - obstacleVelXY
            closingSpeed = max(0.0, -float(np.dot(relativeVelXY, offsetUnit)))
        else:
            closingSpeed = float(np.linalg.norm(np.array(velXY, dtype=float)[:2] - obstacleVelXY))
        return safeRadius + self.egoMovingObstacleTimeBuffer * closingSpeed

    def obstaclePointSafetyMargin(self, posENU=None, t=None):
        if not self.hasObstacles():
            return np.inf, None

        if posENU is None:
            posENU = self.me.mePositionENU
        posENU = np.array(posENU, dtype=float)
        posXY = posENU[:2]
        posZ = posENU[2]
        velXY = np.array(self.me.meVelocityENU, dtype=float)[:2]
        minMargin = np.inf
        closestObstacle = None

        for obstacle in self.currentObstacleStates(t=t):
            zMin, zMax = self.obstacleVerticalRange(obstacle)
            if (
                posZ < zMin - self.egoObstacleVerticalMargin or
                posZ > zMax + self.egoObstacleVerticalMargin
            ):
                continue

            center = np.array(obstacle.get('centerENU', [0.0, 0.0]), dtype=float)[:2]
            safeRadius = self.obstacleSafeRadius(obstacle, posXY=posXY, velXY=velXY)
            margin = float(np.linalg.norm(posXY - center) - safeRadius)
            if margin < minMargin:
                minMargin = margin
                closestObstacle = obstacle

        return minMargin, closestObstacle

    def obstacleSafetyMarginForVelocity(self, velENU):
        if not self.hasObstacles():
            return np.inf, None

        velENU = self.clipVelocityENU(velENU)
        posENU = np.array(self.me.mePositionENU, dtype=float)
        baseObstacleTime = self.obstacleTime()
        horizon = max(float(self.egoObstacleCheckHorizon), self.tStep)
        dt = max(float(self.egoObstacleCheckDt), self.tStep)
        checkTimes = np.concatenate(([0.0], np.arange(dt, horizon + 0.5 * dt, dt)))
        minMargin = np.inf
        closestObstacle = None

        for tau in checkTimes:
            predictedENU = posENU + velENU * tau
            predictedXY = predictedENU[:2]
            predictedZ = predictedENU[2]
            for obstacle in self.currentObstacleStates(t=baseObstacleTime + tau):
                zMin, zMax = self.obstacleVerticalRange(obstacle)
                if (
                    predictedZ < zMin - self.egoObstacleVerticalMargin or
                    predictedZ > zMax + self.egoObstacleVerticalMargin
                ):
                    continue

                center = np.array(obstacle.get('centerENU', [0.0, 0.0]), dtype=float)[:2]
                safeRadius = self.obstacleSafeRadius(obstacle, posXY=predictedXY, velXY=velENU[:2])
                margin = float(np.linalg.norm(predictedXY - center) - safeRadius)
                if margin < minMargin:
                    minMargin = margin
                    closestObstacle = obstacle

        return minMargin, closestObstacle

    def updateObstacleRiskStatus(self, velENU=None):
        if velENU is None:
            velENU = self.u

        pointMargin, pointObstacle = self.obstaclePointSafetyMargin()
        velocityMargin, velocityObstacle = self.obstacleSafetyMarginForVelocity(velENU)

        obstacle = velocityObstacle
        if pointMargin <= velocityMargin:
            obstacle = pointObstacle

        self.egoLastObstaclePointMargin = pointMargin
        self.egoLastObstacleVelocityMargin = velocityMargin
        self.egoLastObstacleName = ''
        self.egoLastObstacleType = ''
        if obstacle is not None:
            self.egoLastObstacleName = str(obstacle.get('name', ''))
            self.egoLastObstacleType = str(obstacle.get('type', ''))

        return min(pointMargin, velocityMargin), obstacle, pointMargin, velocityMargin

    def obstacleEscapeNominalZ(self):
        if self.egoGoalPointENU is not None:
            return float(self.egoGoalPointENU[2])
        if self.platformHoverTargetZ is not None:
            return float(self.platformHoverTargetZ)
        if self.preparePointENU is not None and len(self.preparePointENU) >= 3:
            return float(self.preparePointENU[2])
        return float(self.me.mePositionENU[2])

    def obstacleRadialAndTangentsXY(self, obstacle):
        center = np.array(obstacle.get('centerENU', [0.0, 0.0]), dtype=float)[:2]
        posXY = np.array(self.me.mePositionENU, dtype=float)[:2]
        radialXY = posXY - center
        radialNorm = float(np.linalg.norm(radialXY))
        if radialNorm <= 1e-4:
            radialUnit = np.array([1.0, 0.0], dtype=float)
        else:
            radialUnit = radialXY / radialNorm

        tangentLeft = np.array([-radialUnit[1], radialUnit[0]], dtype=float)
        tangentRight = np.array([radialUnit[1], -radialUnit[0]], dtype=float)
        return radialUnit, tangentLeft, tangentRight

    def obstacleTangentSignForGoal(self, obstacle):
        radialUnit, tangentLeft, tangentRight = self.obstacleRadialAndTangentsXY(obstacle)
        if self.egoGoalPointENU is None:
            return 1.0

        posXY = np.array(self.me.mePositionENU, dtype=float)[:2]
        goalXY = self.egoGoalPointENU[:2] - posXY
        goalNorm = float(np.linalg.norm(goalXY))
        if goalNorm <= 1e-4:
            return 1.0

        goalUnit = goalXY / goalNorm
        if float(np.dot(tangentRight, goalUnit)) > float(np.dot(tangentLeft, goalUnit)):
            return -1.0
        return 1.0

    def obstacleTangentDirectionXY(self, obstacle, tangentSign=None):
        radialUnit, tangentLeft, tangentRight = self.obstacleRadialAndTangentsXY(obstacle)
        sign = self.egoObstacleEscapeTangentSign if tangentSign is None else tangentSign
        tangent = tangentLeft if sign >= 0.0 else tangentRight
        return tangent, radialUnit

    def obstacleIdentity(self, obstacle):
        if obstacle is None:
            return '', ''
        return str(obstacle.get('type', '?')), str(obstacle.get('name', '?'))

    def obstacleByIdentity(self, obstacleType, obstacleName):
        for obstacle in self.currentObstacleStates():
            currentType, currentName = self.obstacleIdentity(obstacle)
            if currentType == obstacleType and currentName == obstacleName:
                return obstacle
        return None

    def obstaclePointMarginForObstacle(self, obstacle, posENU=None):
        if obstacle is None:
            return np.inf
        if posENU is None:
            posENU = self.me.mePositionENU

        posENU = np.array(posENU, dtype=float)
        zMin, zMax = self.obstacleVerticalRange(obstacle)
        if (
            posENU[2] < zMin - self.egoObstacleVerticalMargin or
            posENU[2] > zMax + self.egoObstacleVerticalMargin
        ):
            return np.inf

        posXY = posENU[:2]
        velXY = np.array(self.me.meVelocityENU, dtype=float)[:2]
        center = np.array(obstacle.get('centerENU', [0.0, 0.0]), dtype=float)[:2]
        safeRadius = self.obstacleSafeRadius(obstacle, posXY=posXY, velXY=velXY)
        return float(np.linalg.norm(posXY - center) - safeRadius)

    def shouldSwitchObstacleEscape(self, currentObstacle, currentMargin, candidateObstacle, candidateMargin):
        if candidateObstacle is None:
            return False

        candidateType, candidateName = self.obstacleIdentity(candidateObstacle)
        if (
            candidateType == self.egoObstacleEscapeObstacleType and
            candidateName == self.egoObstacleEscapeObstacleName
        ):
            return False

        now = self.getTimeNow()
        elapsed = now - self.egoObstacleEscapeStartTime
        sinceSwitch = now - self.egoObstacleEscapeLastSwitchTime
        if elapsed < self.egoObstacleEscapeSwitchMinTime:
            return False
        if sinceSwitch < self.egoObstacleEscapeSwitchCooldown:
            return False

        currentSafe = currentMargin >= self.egoObstacleEscapeReleaseMargin
        candidateDangerous = candidateMargin <= self.egoObstacleEscapeTriggerMargin
        candidateMuchWorse = (
            candidateMargin + self.egoObstacleEscapeSwitchMargin < currentMargin
        )
        return candidateDangerous and (currentSafe or candidateMuchWorse)

    def startObstacleEscape(self, obstacle, reason, margin):
        if not self.egoObstacleEscapeEnabled or obstacle is None:
            return False

        now = self.getTimeNow()
        if now - self.egoObstacleEscapeLastReleaseTime < self.egoObstacleEscapeCooldown:
            return False

        self.egoObstacleEscapeActive = True
        self.egoObstacleEscapeTargetENU = None
        self.egoObstacleEscapeTangentSign = self.obstacleTangentSignForGoal(obstacle)
        self.egoObstacleEscapeStartTime = now
        self.egoObstacleEscapeLastSwitchTime = now
        self.egoObstacleEscapeObstacleType, self.egoObstacleEscapeObstacleName = self.obstacleIdentity(obstacle)
        self.egoObstacleEscapeReason = reason
        self.egoObstacleEscapeEntryMargin = float(margin)
        self.egoObstacleEscapeLastPrintTime = 0.0
        self.egoGoalSettledStartTime = None
        self.egoFinalConvergenceForced = False
        self.egoLastCommand = None
        self.egoLastCommandTime = None
        self.resetSmoothedVelocityCommand(self.me.meVelocityENU)
        print(
            f'Obstacle tangent avoidance started: obstacle={self.egoObstacleEscapeObstacleType}:'
            f'{self.egoObstacleEscapeObstacleName}, reason={reason}, '
            f'margin={margin:.2f}, tangent_sign={self.egoObstacleEscapeTangentSign:.0f}'
        )
        return True

    def releaseObstacleEscape(self, reason):
        self.egoObstacleEscapeActive = False
        self.egoObstacleEscapeLastReleaseTime = self.getTimeNow()
        self.egoObstacleEscapeTargetENU = None
        self.egoObstacleEscapeTangentSign = 1.0
        self.egoObstacleEscapeStartTime = None
        self.egoObstacleEscapeLastSwitchTime = 0.0
        self.egoObstacleEscapeReason = ''
        self.egoObstacleEscapeObstacleName = ''
        self.egoObstacleEscapeObstacleType = ''
        self.egoObstacleEscapeEntryMargin = np.inf
        self.egoPlannerTriggered = False
        self.egoLastCommand = None
        self.egoLastCommandTime = None
        self.egoFinalConvergenceForced = False
        self.egoObstacleEscapeReleaseBlendUntil = (
            self.getTimeNow() + max(self.egoObstacleEscapeReleaseBlendTime, 0.0)
        )
        self.resetEgoGoalProgress()
        self.resetSmoothedVelocityCommand(self.me.meVelocityENU)
        print(f'Obstacle tangent avoidance released: {reason}; EGO planner will be retriggered')

    def maybeStartObstacleEscape(self, candidateVelENU=None, reason='predicted unsafe'):
        if not self.egoObstacleEscapeEnabled or self.egoObstacleEscapeActive:
            return False
        if not self.hasObstacles():
            return False

        if candidateVelENU is None:
            margin, obstacle, pointMargin, velocityMargin = self.updateObstacleRiskStatus(self.u)
        else:
            margin, obstacle, pointMargin, velocityMargin = self.updateObstacleRiskStatus(candidateVelENU)

        pointDanger = pointMargin <= self.egoObstacleEscapeTriggerMargin
        velocityDanger = velocityMargin <= self.egoObstacleEscapeVelocityTriggerMargin
        if not pointDanger and not velocityDanger:
            return False

        boundaryWarning = self.safetyBoundaryViolation(margin=self.egoSafetyMargin)
        if boundaryWarning and not self.safetyBoundaryIsHeight(boundaryWarning):
            return False

        startReason = 'predicted obstacle safety envelope' if velocityDanger else 'inside obstacle safety envelope'
        return self.startObstacleEscape(obstacle, startReason, margin)

    def stepObstacleEscape(self):
        if not self.egoObstacleEscapeActive:
            return False

        if not self.ensureGuidanceOffboardControl():
            return True

        pointMargin, pointObstacle = self.obstaclePointSafetyMargin()
        obstacle = self.obstacleByIdentity(
            self.egoObstacleEscapeObstacleType,
            self.egoObstacleEscapeObstacleName
        )
        if obstacle is None:
            obstacle = pointObstacle

        if obstacle is None:
            self.releaseObstacleEscape('obstacle no longer available')
            return False

        trackedMargin = self.obstaclePointMarginForObstacle(obstacle)
        obstacleType, obstacleName = self.obstacleIdentity(obstacle)
        if (
            obstacleType != self.egoObstacleEscapeObstacleType or
            obstacleName != self.egoObstacleEscapeObstacleName
        ):
            self.egoObstacleEscapeObstacleType = obstacleType
            self.egoObstacleEscapeObstacleName = obstacleName
            self.egoObstacleEscapeTangentSign = self.obstacleTangentSignForGoal(obstacle)
            self.egoObstacleEscapeEntryMargin = float(pointMargin)
            self.egoObstacleEscapeLastSwitchTime = self.getTimeNow()
            print(
                f'Obstacle tangent avoidance switched to closest danger: '
                f'obstacle={obstacleType}:{obstacleName}, margin={pointMargin:.2f}, '
                f'tangent_sign={self.egoObstacleEscapeTangentSign:.0f}'
            )
        elif self.shouldSwitchObstacleEscape(obstacle, trackedMargin, pointObstacle, pointMargin):
            previousMargin = trackedMargin
            obstacle = pointObstacle
            trackedMargin = pointMargin
            obstacleType, obstacleName = self.obstacleIdentity(obstacle)
            self.egoObstacleEscapeObstacleType = obstacleType
            self.egoObstacleEscapeObstacleName = obstacleName
            self.egoObstacleEscapeTangentSign = self.obstacleTangentSignForGoal(obstacle)
            self.egoObstacleEscapeEntryMargin = float(pointMargin)
            self.egoObstacleEscapeLastSwitchTime = self.getTimeNow()
            print(
                f'Obstacle tangent avoidance switched to significantly closer danger: '
                f'obstacle={obstacleType}:{obstacleName}, current_margin={previousMargin:.2f}, '
                f'closest_margin={pointMargin:.2f}, tangent_sign={self.egoObstacleEscapeTangentSign:.0f}'
            )
        else:
            pointMargin = trackedMargin

        tangentUnit, radialUnit = self.obstacleTangentDirectionXY(obstacle)
        tangentialSpeed = min(self.egoObstacleEscapeSpeed, self.egoMaxControlSpeed)
        velENU = np.zeros(3, dtype=float)
        velENU[:2] = tangentialSpeed * tangentUnit

        if self.egoGoalPointENU is not None and self.egoObstacleEscapeGoalBias > 0.0:
            goalXY = self.egoGoalPointENU[:2] - self.me.mePositionENU[:2]
            goalNorm = float(np.linalg.norm(goalXY))
            if goalNorm > 1e-4:
                goalVelXY = self.egoObstacleEscapeGoalBias * goalXY / goalNorm
                inward = float(np.dot(goalVelXY, radialUnit))
                if inward < 0.0:
                    goalVelXY -= inward * radialUnit
                velENU[:2] += goalVelXY

        if trackedMargin < 0.0:
            velENU[:2] += self.egoObstacleEscapeAwayGain * abs(trackedMargin) * radialUnit

        elapsed = self.getTimeNow() - self.egoObstacleEscapeStartTime
        if self.egoObstacleEscapeRampTime > 1e-3:
            ramp = float(np.clip(elapsed / self.egoObstacleEscapeRampTime, 0.25, 1.0))
            velENU[:2] *= ramp

        targetZ = self.obstacleEscapeNominalZ()
        zError = targetZ - self.me.mePositionENU[2]
        if abs(zError) <= max(self.egoGoalZTolerance, 0.12):
            velENU[2] = -0.08 * self.me.meVelocityENU[2]
        else:
            velENU[2] = 0.22 * zError - 0.10 * self.me.meVelocityENU[2]

        velENU = self.clipVelocityENU(
            velENU,
            maxSpeed=self.egoObstacleEscapeSpeed,
            maxVerticalSpeed=self.egoObstacleEscapeVerticalSpeed
        )
        velENU = self.sendBoundedVelocityENUControl(
            velENU,
            self.yawRadENU,
            useObstacleGuard=False,
            smooth=True
        )

        margin, _obstacle, pointMargin, velocityMargin = self.updateObstacleRiskStatus(velENU)
        timeoutReached = elapsed >= self.egoObstacleEscapeTimeout

        now = self.getTimeNow()
        if now - self.egoObstacleEscapeLastPrintTime >= 0.5:
            print(
                f'Obstacle tangent avoidance active: obstacle={self.egoObstacleEscapeObstacleType}:'
                f'{self.egoObstacleEscapeObstacleName}, point_margin={pointMargin:.2f}, '
                f'vel_margin={velocityMargin:.2f}, target_z={targetZ:.2f}, '
                f'tangent_sign={self.egoObstacleEscapeTangentSign:.0f}, cmd={arrayString(velENU)}'
            )
            self.egoObstacleEscapeLastPrintTime = now

        releaseSafe = (
            elapsed >= self.egoObstacleEscapeMinTime and
            pointMargin >= self.egoObstacleEscapeReleaseMargin and
            velocityMargin >= self.egoObstacleEscapeReleaseMargin
        )
        if releaseSafe:
            self.releaseObstacleEscape(
                f'left danger zone, point_margin={pointMargin:.2f}, '
                f'vel_margin={velocityMargin:.2f}'
            )
        elif timeoutReached:
            self.egoObstacleEscapeStartTime = self.getTimeNow()
            print(
                f'Obstacle tangent avoidance continuing after timeout: '
                f'point_margin={pointMargin:.2f}, release={self.egoObstacleEscapeReleaseMargin:.2f}'
            )

        return True

    def obstacleVelocityIsSafe(self, velENU):
        minMargin, _obstacle = self.obstacleSafetyMarginForVelocity(velENU)
        return minMargin >= 0.0

    def rotatedXYVelocity(self, speed, angleRad, verticalVelocity):
        return np.array([
            speed * math.cos(angleRad),
            speed * math.sin(angleRad),
            verticalVelocity
        ], dtype=float)

    def obstacleCandidateVelocities(self, targetVelENU, closestObstacle=None):
        targetVelENU = self.clipVelocityENU(targetVelENU)
        targetXY = targetVelENU[:2]
        targetXYSpeed = float(np.linalg.norm(targetXY))
        sampleSpeed = targetXYSpeed if targetXYSpeed > 1e-4 else self.egoObstacleFallbackSpeed
        sampleSpeed = min(sampleSpeed, self.egoMaxControlSpeed)

        def angleFromXY(vectorXY):
            norm = float(np.linalg.norm(vectorXY))
            if norm <= 1e-4:
                return None
            return math.atan2(vectorXY[1], vectorXY[0])

        def appendAngle(angles, angle):
            if angle is None:
                return
            for existing in angles:
                delta = math.atan2(math.sin(angle - existing), math.cos(angle - existing))
                if abs(delta) < 1e-3:
                    return
            angles.append(angle)

        baseAngles = []
        appendAngle(baseAngles, angleFromXY(targetXY))

        if self.egoGoalPointENU is not None:
            goalXY = self.egoGoalPointENU[:2] - self.me.mePositionENU[:2]
            appendAngle(baseAngles, angleFromXY(goalXY))

        if closestObstacle is not None:
            center = np.array(closestObstacle.get('centerENU', [0.0, 0.0]), dtype=float)[:2]
            offset = self.me.mePositionENU[:2] - center
            obstacleAngle = angleFromXY(offset)
            if obstacleAngle is not None:
                appendAngle(baseAngles, obstacleAngle + math.pi / 2.0)
                appendAngle(baseAngles, obstacleAngle - math.pi / 2.0)

        if not baseAngles:
            baseAngles = [0.0]

        candidates = [np.zeros(3, dtype=float), targetVelENU]
        for base in baseAngles:
            for scale in self.egoObstacleCandidateSpeedScales:
                speed = max(0.0, float(scale)) * sampleSpeed
                for deltaDeg in self.egoObstacleCandidateAnglesDeg:
                    angle = base + math.radians(float(deltaDeg))
                    candidate = self.rotatedXYVelocity(speed, angle, targetVelENU[2])
                    candidates.append(self.clipVelocityENU(candidate))

        if closestObstacle is not None and closestObstacle.get('type') == 'moving':
            obstacleVelXY = np.array(
                closestObstacle.get('velocityENU', [0.0, 0.0, 0.0]),
                dtype=float
            )[:2]
            center = np.array(closestObstacle.get('centerENU', [0.0, 0.0]), dtype=float)[:2]
            offset = self.me.mePositionENU[:2] - center
            relativeTargetXY = targetVelENU[:2] - obstacleVelXY
            relativeTargetSpeed = float(np.linalg.norm(relativeTargetXY))
            relativeSpeed = relativeTargetSpeed if relativeTargetSpeed > 1e-4 else self.egoObstacleFallbackSpeed
            relativeSpeed = min(relativeSpeed, self.egoMaxControlSpeed)

            movingAngles = []
            awayAngle = angleFromXY(offset)
            if awayAngle is not None:
                appendAngle(movingAngles, awayAngle)
                appendAngle(movingAngles, awayAngle + math.pi / 4.0)
                appendAngle(movingAngles, awayAngle - math.pi / 4.0)
                appendAngle(movingAngles, awayAngle + math.pi / 2.0)
                appendAngle(movingAngles, awayAngle - math.pi / 2.0)
            appendAngle(movingAngles, angleFromXY(relativeTargetXY))

            candidates.append(self.clipVelocityENU(np.array([
                obstacleVelXY[0],
                obstacleVelXY[1],
                max(targetVelENU[2], 0.0) if self.egoObstacleBlockDescentWhenUnsafe else targetVelENU[2]
            ], dtype=float)))

            for base in movingAngles or baseAngles:
                for scale in self.egoObstacleCandidateSpeedScales:
                    relativeSpeedScaled = max(0.0, float(scale)) * relativeSpeed
                    for deltaDeg in self.egoObstacleCandidateAnglesDeg:
                        angle = base + math.radians(float(deltaDeg))
                        relativeXY = np.array([
                            relativeSpeedScaled * math.cos(angle),
                            relativeSpeedScaled * math.sin(angle),
                        ], dtype=float)
                        candidate = np.array([
                            obstacleVelXY[0] + relativeXY[0],
                            obstacleVelXY[1] + relativeXY[1],
                            targetVelENU[2],
                        ], dtype=float)
                        candidates.append(self.clipVelocityENU(candidate))

        return candidates

    def obstacleVelocityScore(self, candidateVelENU, targetVelENU, closestObstacle=None):
        targetVelENU = np.array(targetVelENU, dtype=float)
        candidateVelENU = np.array(candidateVelENU, dtype=float)
        score = float(np.linalg.norm(candidateVelENU - targetVelENU))

        if self.egoGoalPointENU is not None:
            goalXY = self.egoGoalPointENU[:2] - self.me.mePositionENU[:2]
            goalNorm = float(np.linalg.norm(goalXY))
            candNorm = float(np.linalg.norm(candidateVelENU[:2]))
            if goalNorm > 1e-4 and candNorm > 1e-4:
                goalUnit = goalXY / goalNorm
                progressSpeed = float(np.dot(candidateVelENU[:2], goalUnit))
                score -= self.egoObstacleGoalBias * progressSpeed
            elif candNorm <= 1e-4:
                score += self.egoObstacleStopPenalty

        if closestObstacle is not None and closestObstacle.get('type') == 'moving':
            center = np.array(closestObstacle.get('centerENU', [0.0, 0.0]), dtype=float)[:2]
            offset = self.me.mePositionENU[:2] - center
            offsetNorm = float(np.linalg.norm(offset))
            if offsetNorm > 1e-4:
                offsetUnit = offset / offsetNorm
                obstacleVelXY = np.array(
                    closestObstacle.get('velocityENU', [0.0, 0.0, 0.0]),
                    dtype=float
                )[:2]
                relativeVelXY = candidateVelENU[:2] - obstacleVelXY
                radialSpeed = float(np.dot(relativeVelXY, offsetUnit))
                if radialSpeed < 0.0:
                    score += self.egoMovingObstacleClosingPenalty * abs(radialSpeed)
                else:
                    score -= self.egoMovingObstacleAwayBias * radialSpeed

        return score

    def projectVelocityAwayFromObstacles(self, velENU):
        if not self.hasObstacles():
            return np.array(velENU, dtype=float)

        safeVelENU = np.array(velENU, dtype=float)
        margin, closestObstacle = self.obstacleSafetyMarginForVelocity(safeVelENU)
        if margin >= 0.0:
            return safeVelENU

        targetVelENU = np.array(safeVelENU, dtype=float)
        if self.egoObstacleBlockDescentWhenUnsafe:
            targetVelENU[2] = max(targetVelENU[2], 0.0)

        bestVelENU = None
        bestScore = np.inf
        bestMargin = -np.inf
        for candidateVelENU in self.obstacleCandidateVelocities(targetVelENU, closestObstacle=closestObstacle):
            candidateMargin, _obstacle = self.obstacleSafetyMarginForVelocity(candidateVelENU)
            if candidateMargin < 0.0:
                if candidateMargin > bestMargin:
                    bestMargin = candidateMargin
                    bestVelENU = candidateVelENU
                continue

            score = self.obstacleVelocityScore(candidateVelENU, safeVelENU, closestObstacle=closestObstacle)
            if score < bestScore:
                bestScore = score
                bestVelENU = candidateVelENU
                bestMargin = candidateMargin

        if bestVelENU is None:
            bestVelENU = np.zeros(3, dtype=float)
            bestMargin = margin

        safeVelENU = self.clipVelocityENU(bestVelENU)
        obstacleName = ''
        if closestObstacle is not None:
            obstacleName = f" near obstacle {closestObstacle.get('name', '?')}"
        print(
            f'Obstacle safety filter adjusted velocity{obstacleName}: '
            f'margin {margin:.2f} -> {bestMargin:.2f}, '
            f'cmd {arrayString(velENU)} -> {arrayString(safeVelENU)}'
        )

        return safeVelENU

    def guardedVelocityToPointENUControl(self, pointENU, yawRadENU):
        velENU = self.me.kp * (pointENU - self.me.mePositionENU)
        self.sendBoundedVelocityENUControl(velENU, yawRadENU, useObstacleGuard=True)

    def platformFinalHoverPointENU(self):
        if self.platformLandOnTop:
            return None

        configuredGoalPointENU = self.configuredGoalPointENU()
        if configuredGoalPointENU is not None:
            return configuredGoalPointENU

        if self.egoGoalPointENU is not None:
            return np.array(self.egoGoalPointENU, dtype=float)

        goals = self.egoPlannerConfig.get('goalsENU', [])
        if len(goals) >= self.number and len(goals[self.number - 1]) >= 2:
            hoverZ = self.platformDefaultHoverReferenceZ()
            if hoverZ is None:
                return None
            return np.array([
                goals[self.number - 1][0],
                goals[self.number - 1][1],
                hoverZ,
            ], dtype=float)

        platformCenter = self.platformData.get('centerENU')
        if platformCenter is not None and len(platformCenter) >= 2:
            hoverZ = self.platformDefaultHoverReferenceZ()
            if hoverZ is None:
                return None
            return np.array([
                float(platformCenter[0]),
                float(platformCenter[1]),
                hoverZ,
            ], dtype=float)

        return None

    def switchToPlatformFinalHoverIfNeeded(self):
        if self.platformFinalHoverActive:
            return False

        finalHoverPointENU = self.platformFinalHoverPointENU()
        if finalHoverPointENU is None or self.egoGoalPointENU is None:
            return False

        if np.linalg.norm(finalHoverPointENU - self.egoGoalPointENU) <= 1e-3:
            self.platformFinalHoverActive = True
            return False

        oldGoalPointENU = np.array(self.egoGoalPointENU, dtype=float)
        self.egoGoalPointENU = finalHoverPointENU
        self.platformFinalHoverActive = True
        self.egoGoalReached = False
        self.stateFinished = False
        self.egoGoalSettledStartTime = None
        self.egoAllFinishedStartTime = None
        self.resetEgoGoalProgress()
        self.egoFinalConvergenceForced = True
        print(
            f'Platform approach target settled = {arrayString(oldGoalPointENU)}; '
            f'switching to final hover target = {arrayString(self.egoGoalPointENU)}'
        )
        return True

    def platformFinalHoverSwitchXYTolerance(self):
        return float(self.platformData.get(
            'hoverSwitchXYTolerance',
            self.egoGoalXYTolerance
        ))

    def egoControlPointENU(self):
        if self.egoGoalPointENU is None:
            return None
        return np.array(self.egoGoalPointENU, dtype=float)

    def holdEgoGoalPoint(self):
        if self.egoGoalPointENU is None:
            self.holdCurrentPosition('missing EGO final target')
            return
        self.convergeToEgoGoalPoint(useObstacleGuard=False)

    def convergeToEgoGoalPoint(self, useObstacleGuard=False):
        posErrorENU = self.egoGoalPointENU - self.me.mePositionENU
        xyError = np.linalg.norm(posErrorENU[:2])
        zError = abs(posErrorENU[2])
        velocityLimit = self.egoHoldVelocityLimit
        verticalVelocityLimit = min(self.egoMaxVerticalSpeed, velocityLimit)
        nearZDistance = max(2.0 * self.egoGoalZTolerance, 0.2)
        if xyError <= self.egoHoldNearDistance and zError <= nearZDistance:
            velocityLimit = self.egoHoldNearVelocityLimit
            verticalVelocityLimit = self.egoHoldNearVerticalVelocityLimit

        desiredVelENU = np.zeros(3)
        desiredVelENU[:2] = self.egoHoldXYGain * posErrorENU[:2]
        desiredVelENU[2] = self.egoHoldZGain * posErrorENU[2]
        desiredVelENU[:2] = self.clipVectorNorm(desiredVelENU[:2], velocityLimit)
        desiredVelENU[2] = np.clip(
            desiredVelENU[2],
            -verticalVelocityLimit,
            verticalVelocityLimit
        )
        totalVelocityLimit = max(velocityLimit, verticalVelocityLimit)
        desiredVelENU = self.clipVelocityENU(
            desiredVelENU,
            maxSpeed=totalVelocityLimit,
            maxVerticalSpeed=verticalVelocityLimit
        )
        controlVelENU = np.array(desiredVelENU, dtype=float)
        controlVelENU[:2] += self.egoHoldVelocityGain * (
            desiredVelENU[:2] - self.me.meVelocityENU[:2]
        )
        controlVelENU = self.clipVelocityENU(
            controlVelENU,
            maxSpeed=totalVelocityLimit,
            maxVerticalSpeed=verticalVelocityLimit
        )
        self.sendBoundedVelocityENUControl(
            controlVelENU,
            self.yawRadENU,
            useObstacleGuard=useObstacleGuard,
            smooth=not (xyError <= self.egoHoldNearDistance and zError <= nearZDistance)
        )

    def egoGoalErrorComponents(self):
        if self.egoGoalPointENU is None:
            return np.inf, np.inf, np.inf
        posErrorENU = self.egoGoalPointENU - self.me.mePositionENU
        xyError = np.linalg.norm(posErrorENU[:2])
        zError = abs(posErrorENU[2])
        speed = np.linalg.norm(self.me.meVelocityENU)
        return xyError, zError, speed

    def egoGoalDistance(self):
        if self.egoGoalPointENU is None:
            return np.inf
        return float(np.linalg.norm(self.egoGoalPointENU - self.me.mePositionENU))

    def resetEgoGoalProgress(self):
        self.egoBestGoalDistance = self.egoGoalDistance()
        self.egoBestGoalDistanceTime = self.getTimeNow()
        self.egoFinalConvergenceForced = False
        self.egoLastProgressFallbackPrintTime = 0.0

    def updateEgoGoalProgress(self):
        distance = self.egoGoalDistance()
        now = self.getTimeNow()
        if (
            self.egoBestGoalDistanceTime is None or
            distance < self.egoBestGoalDistance - self.egoProgressMinImprovement
        ):
            self.egoBestGoalDistance = distance
            self.egoBestGoalDistanceTime = now
        return distance

    def egoGoalProgressStalled(self, distance):
        if self.egoGoalPointENU is None:
            return False
        if self.stateTime < self.egoProgressFallbackGraceTime:
            return False
        if distance <= self.egoProgressFallbackDistance:
            return False
        if self.egoBestGoalDistanceTime is None:
            return False
        return self.getTimeNow() - self.egoBestGoalDistanceTime >= self.egoProgressFallbackTime

    def egoGoalIsSettled(self):
        xyError, zError, speed = self.egoGoalErrorComponents()
        if self.platformFinalHoverActive and not self.platformLandOnTop:
            return (
                xyError <= self.egoGoalXYTolerance and
                zError <= self.egoGoalZTolerance and
                speed <= self.egoGoalSpeedTolerance
            )
        return (
            xyError <= self.egoGoalXYTolerance and
            zError <= self.egoGoalZTolerance and
            speed <= self.egoGoalSpeedTolerance
        )

    def velocityToPointENUControl(self, pointENU, yawRadENU):
        velENU = self.me.kp * (pointENU - self.me.mePositionENU)
        self.u = velENU
        self.me.velocityENUControl(velENU, yawRadENU)

    def guardedConstantVelocityLineENUControl(self, constantVelocityENU, positionENU, yawRadENU):
        kI = 0.05
        kp = 0.5
        velENU = constantVelocityENU + kI * (positionENU - self.me.mePositionENU) + kp * (constantVelocityENU - self.me.meVelocityENU)
        self.u = velENU
        self.me.velocityENUControl(velENU, yawRadENU)

    def isLeader(self):
        return self.number <= self.config['leaders']
    
    def getName(self):
        if self.isLeader():
            return f'Leader_{self.number}'
        else:
            return f'Follower_{self.number - self.config["leaders"]}'

    def number2Name(self, number, fullName=False):
        if number <= self.config["leaders"]:
            if fullName:
                return f'Leader_{number}'
            else:
                return f'l{number}'
        else:
            if fullName:
                return f'Follower_{number - self.config["leaders"]}'
            else:
                return f'f{number - self.config["leaders"]}'
            
    def name2Number(self, name):
        if name.startswith('l'):
            return eval(name[1:])
        elif name.startswith('f'):
            return eval(name[1:]) + self.config["leaders"]
        else:
            raise Exception(f'Invalid name: {name}')
    def isHead(self):
        return self.number == self.config['head']
    
    def othersAllAtState(self, state: State):
        return all([self.communicator.othersInfo[uav_number]['state'] == state.name for uav_number in range(1, self.config['number'] + 1) if uav_number != self.number])
    
    def othersAllAtFinishedState(self, state: State):
        return all([self.communicator.othersInfo[uav_number]['state'] == state.name and self.communicator.othersInfo[uav_number]['stateFinished'] == True for uav_number in range(1, self.config['number'] + 1) if uav_number != self.number])
    
    def headAtState(self, state: State):
        return self.communicator.othersInfo[self.config['head']]['state'] == state.name

    def allGuidanceGoalsFinished(self):
        if not self.stateFinished:
            return False
        for uav_number in range(1, self.config['number'] + 1):
            if uav_number == self.number:
                continue
            info = self.communicator.othersInfo[uav_number]
            if info['state'] in (State.LAND.name, State.END.name):
                continue
            if info['state'] != State.GUIDANCE.name or info['stateFinished'] != True:
                return False
        return True

    def getTimeNow(self):
        return time.time()

    @stepEntrance
    def toStepTakeoff(self):
        self.state = State.TAKEOFF
        self.me.setPositionControlMode()
        self.takeoffVerticalPointENU = np.array([
            self.me.mePositionENU[0],
            self.me.mePositionENU[1],
            self.takeoffPointENU[2]
        ])

    @stepEntrance
    def toStepAttitudeBiasTest(self):
        self.state = State.ATTITUDE_BIAS_TEST
        self.rollRecords = []
        self.pitchRecords = []

    @stepEntrance
    def toStepThrottleTest(self):
        self.state = State.THROTTLE_TEST
        self.throttleMin = self.throttleTestMin
        self.throttleMax = self.throttleTestMax
        self.throttle = (self.throttleMax + self.throttleMin) / 2.0
        self.changeTime = self.throttleTestChangeTime
        self.changeStep = self.throttleTestChangeTime
        self.throttleTestAdjustPosition = False
        self.lastVerVel = self.me.meVelocityENU[2]
        self.me.setAttitudeControlMode()

    @stepEntrance
    def toStepPrepare(self):
        self.state = State.PREPARE
        self.me.setPositionControlMode()

    @stepEntrance
    def toStepGuidance(self):
        self.state = State.GUIDANCE
        self.egoPlannerTriggered = False
        self.egoGoalReached = False
        self.egoGoalSettledStartTime = None
        self.egoAllFinishedStartTime = None
        self.platformFinalHoverActive = False
        self.egoObstacleEscapeActive = False
        self.egoObstacleEscapeTargetENU = None
        self.egoObstacleEscapeTangentSign = 1.0
        self.egoObstacleEscapeStartTime = None
        self.egoObstacleEscapeReleaseBlendUntil = 0.0
        if self.egoGoalPointENU is not None:
            self.resetEgoGoalProgress()
        if self.egoPlannerEnabled and self.egoPlannerAvailable:
            self.me.setVelocityControlMode()
            self.resetSmoothedVelocityCommand(self.me.meVelocityENU)
        else:
            self.me.setAccelerationControlMode()

    @stepEntrance
    def toStepBack(self):
        self.state = State.BACK
        self.me.setPositionControlMode()

    @stepEntrance
    def toStepLand(self):
        self.state = State.LAND
        self.me.setVelocityControlMode()
        self.landCommandSent = False
        self.lastLandCommandTime = 0.0
        self.platformLandingSettleStartTime = None

    @stepEntrance
    def toStepEnd(self):
        self.state = State.END

    def isThisStateFinished(self):
        return self.stateFinished

    def finishState(self):
        self.stateFinished = True

    def holdCurrentPosition(self, reason=''):
        currentPositionENU = np.array(self.me.mePositionENU, dtype=float)
        self.u = np.zeros(3)
        self.me.positionENUControl(currentPositionENU, self.yawRadENU)
        if reason:
            print(f'Holding current position: {reason}')

    def ensureGuidanceOffboardControl(self):
        mode = self.me.meState.mode
        armed = self.me.isArmed()
        if armed and mode == 'OFFBOARD':
            return True

        if armed:
            recoveryReason = self.safetyBoundaryViolation(margin=self.egoSafetyMargin)
            recoveryVelENU = self.safetyRecoveryVelocity(reason=recoveryReason or None)
            recoveryVelENU -= 0.4 * self.me.meVelocityENU
            recoveryVelENU = self.clipVelocityENU(
                recoveryVelENU,
                maxVerticalSpeed=self.egoSafetyRecoveryVerticalLimit
            )
            self.sendBoundedVelocityENUControl(
                recoveryVelENU,
                self.yawRadENU,
                useObstacleGuard=False,
                smooth=False
            )
            print(
                f'PX4 control not ready: armed={armed}, mode={mode}; '
                f'publishing recovery velocity {arrayString(recoveryVelENU)}'
            )
        else:
            self.holdCurrentPosition(f'PX4 control not ready: armed={armed}, mode={mode}')

        now = self.getTimeNow()
        if now - self.lastOffboardRecoveryAttemptTime < self.egoOffboardRecoveryPeriod:
            return False

        self.lastOffboardRecoveryAttemptTime = now
        self.me.setVelocityControlMode()
        if not armed:
            self.me.arm()
        if mode != 'OFFBOARD':
            self.me.intoOffboardMode()
        return False

    def stepInit(self):
        if not self.me.intoOffboardMode():
            return
        if self.me.meState.mode == 'OFFBOARD':
            self.finishState()
        if not self.takeoff:
            if not self.me.hasSetOrigin():
                self.me.setGpsOrigin()
                return
        if self.isThisStateFinished():
            if self.isHead():
                if self.othersAllAtFinishedState(State.INIT):
                    self.toStepTakeoff()
                else:
                    print(f'Waiting for others to enter state {State.INIT.name}')
            else:
                if self.headAtState(State.TAKEOFF):
                    self.toStepTakeoff()
                else:
                    print(f'Waiting for UAV #{self.config["head"]} to enter state {State.TAKEOFF.name}')

    def stepTakeoff(self):
        if not self.me.isArmed():
            self.me.arm()
        if self.takeoffVerticalPointENU is None:
            self.takeoffVerticalPointENU = np.array([
                self.me.mePositionENU[0],
                self.me.mePositionENU[1],
                self.takeoffPointENU[2]
            ])

        self.velocityToPointENUControl(self.takeoffVerticalPointENU, self.yawRadENU)

        if self.me.nearPositionENU(self.takeoffVerticalPointENU) and self.me.nearSpeed(0.0):
            self.finishState()

        if self.isThisStateFinished():
            if self.isHead():
                if self.othersAllAtFinishedState(State.TAKEOFF):
                    self.toStepPrepare()
                else:
                    print(f'Waiting for others to finish {State.TAKEOFF.name}')
            else:
                if self.headAtState(State.PREPARE):
                    self.toStepPrepare()
                else:
                    print(f'Waiting for UAV #{self.config["head"]} to enter state {State.PREPARE.name}')

    def stepAttitudeBiasTest(self):
        self.me.hoverWithYaw(self.yawRadENU)
        if self.me.nearSpeed(0.0, tol=0.1):
            self.rollRecords.append(self.me.meRPYRadENU[0])
            self.pitchRecords.append(self.me.meRPYRadENU[1])
        print(f'{len(self.rollRecords) = }, {len(self.pitchRecords) = }')
        if self.stateTime >= 20.0 or (len(self.rollRecords) > 100 and len(self.pitchRecords) > 100):
            self.me.rollOffsetRad = np.mean(self.rollRecords)
            self.me.pitchOffsetRad = np.mean(self.pitchRecords)
            self.addMessage(f'Roll offset: {np.rad2deg(self.me.rollOffsetRad):.3f} deg, pitch offset: {np.rad2deg(self.me.pitchOffsetRad):.3f} deg')
            self.toStepThrottleTest()

    def stepThrottleTest(self):
        if self.stateTime >= 100.0:
            self.toStepLand()
            return
        
        if not self.safetyModule():
            self.toStepLand()
            return
        
        self.throttle = (self.throttleMin + self.throttleMax) / 2.0
        print(f'Between {self.throttleMin:.3f} and {self.throttleMax:.3f}: try {self.throttle:.3f}')
        print(f'Wait for {self.changeTime:.2f} to change, step is {self.changeStep:.2f}')
        print(f'Last vertical velocity @ {self.lastVerVel:.2f}')
        if self.stateTime >= self.changeTime:
            print(f'Now Vertical velocity end @ {self.me.meVelocityENU[2]:.2f}')
            if self.me.meVelocityENU[2] < self.lastVerVel:
                print(f'Hover throttle too low')
                self.throttleMin = self.throttle
            else:
                print(f'Hover throttle too high')
                self.throttleMax = self.throttle
            self.changeTime += self.changeStep
        self.lastVerVel = self.me.meVelocityENU[2]
                
        if self.throttleMax - self.throttleMin < self.throttleTestAccuracy:
            print(f'Throttle test result: between {self.throttleMin} and {self.throttleMax}')
            self.me.hoverThrottle = self.throttleMax
            self.toStepPrepare()
            return

        if self.throttleMax - self.throttleTestMin < 0.01:
            print(f'Throttle test failed: range to high')
            self.toStepLand()
            return

        if self.throttleTestMax - self.throttleMin < 0.01:
            print(f'Throttle test failed: range to low')
            self.toStepLand()
            return

        self.me.rpyENUThrustControl([0, 0, self.yawRadENU], self.throttle)

    def stepPrepare(self):
        print(f'Prepare to {arrayString(self.preparePointENU)}')
        self.velocityToPointENUControl(self.preparePointENU, self.yawRadENU)
        if self.me.nearPositionENU(self.preparePointENU) and self.me.nearSpeed(0.0):
            self.finishState()

        if self.isThisStateFinished():
            if self.isHead():
                if self.othersAllAtFinishedState(State.PREPARE):
                    self.toStepGuidance()
                else:
                    print(f'Waiting for others to enter state {State.PREPARE.name}')
            else:
                if self.headAtState(State.GUIDANCE):
                    self.toStepGuidance()
                else:
                    print(f'Waiting for UAV #{self.config["head"]} to enter state {State.GUIDANCE.name}')

    def stepGuidance(self):
        if self.egoPlannerEnabled and self.egoPlannerAvailable:
            self.stepEgoPlannerGuidance()
            return

        if self.isLeader():
            self.guardedConstantVelocityLineENUControl(
                self.leadervelocityENU,
                self.preparePointENU + self.stateTime * self.leadervelocityENU,
                self.yawRadENU
            )
        else:
            if not self.safetyModule():
                self.holdCurrentPosition('safety module blocked fallback guidance')
                return
    
            self.u = np.array([0.0, -1.0, 0.0])
            print(f"guidanceCommandENU = {arrayString(self.u)}")

            thrust, self.cmdRPYRadENU = accENUYawENU2EulerENUThrust(
                accENU=self.u, 
                yawRadENU=self.yawRadENU, 
                hoverThrottle=self.me.hoverThrottle
            )
            self.me.rpyENUThrustControl(self.cmdRPYRadENU, thrust)

        if self.stateTime > self.formationTime:
            self.holdCurrentPosition('formation time elapsed in fallback guidance')

    def stepEgoPlannerGuidance(self):
        if not self.ensureGuidanceOffboardControl():
            return

        if self.egoGoalReached:
            if self.allGuidanceGoalsFinished():
                now = self.getTimeNow()
                if self.egoAllFinishedStartTime is None:
                    self.egoAllFinishedStartTime = now
                self.holdEgoGoalPoint()
                waitTime = now - self.egoAllFinishedStartTime
                if self.platformLandOnTop:
                    print(
                        f'All UAVs reached platform approach targets; landing on platform '
                        f'in {max(self.egoEndDelay - waitTime, 0.0):.2f}s'
                    )
                    if waitTime >= self.egoEndDelay:
                        self.toStepLand()
                else:
                    print(
                        f'All UAVs reached final hover targets; holding at '
                        f'{arrayString(self.egoGoalPointENU)}'
                    )
                return
            self.egoAllFinishedStartTime = None
            self.holdEgoGoalPoint()
            xyError, zError, speed = self.egoGoalErrorComponents()
            print(
                f'Holding final target = {arrayString(self.egoGoalPointENU)}, '
                f'xy err = {xyError:.2f}, z err = {zError:.2f}, speed = {speed:.2f}'
            )
            return

        boundaryViolation = self.safetyBoundaryViolation(margin=0.0)
        if boundaryViolation and (
            self.plannerBackend == 'egov2' or
            not self.safetyBoundaryIsHeight(boundaryViolation)
        ):
            self.recoverFromSafetyBoundary(boundaryViolation)
            return

        hardSafetyViolation = self.guidanceHardSafetyViolation()
        if hardSafetyViolation:
            self.recoverFromGuidanceSafety()
            return

        if self.stepObstacleEscape():
            return

        self.updateObstacleRiskStatus(self.u)
        if self.maybeStartObstacleEscape(self.u, reason='current obstacle risk'):
            self.stepObstacleEscape()
            return

        self.publishEgoStartTrigger()

        currentSpeed = float(np.linalg.norm(self.me.meVelocityENU))
        if currentSpeed > self.egoVelocitySafetyLimit:
            brakingVelENU = self.clipVelocityENU(-0.6 * self.me.meVelocityENU)
            boundaryWarning = self.safetyBoundaryViolation(margin=self.egoSafetyMargin)
            if boundaryWarning == 'too low':
                brakingVelENU[2] = max(brakingVelENU[2], self.egoSafetyRecoveryVerticalLimit)
            elif boundaryWarning == 'too high':
                brakingVelENU[2] = min(brakingVelENU[2], -self.egoSafetyRecoveryVerticalLimit)
            self.sendBoundedVelocityENUControl(
                brakingVelENU,
                self.yawRadENU,
                useObstacleGuard=False,
                smooth=False
            )
            print(
                f'EGO speed guard active: speed={currentSpeed:.2f}, '
                f'limit={self.egoVelocitySafetyLimit:.2f}, '
                f'braking cmd = {arrayString(brakingVelENU)}'
            )
            return

        commandFresh = (
            self.egoLastCommand is not None and
            self.egoLastCommandTime is not None and
            self.getTimeNow() - self.egoLastCommandTime <= self.egoCommandTimeout
        )
        goalDistance = self.updateEgoGoalProgress() if self.egoGoalPointENU is not None else np.inf
        nearFinalTarget = goalDistance <= self.egoFinalConvergenceDistance

        noCommandFallback = (
            self.egoGoalPointENU is not None and
            self.stateTime > self.egoTriggerDelay + self.egoNoCommandFallbackDelay
        )
        finalConvergenceTimedOut = self.stateTime >= self.egoFinalConvergenceTime
        progressStalled = self.egoProgressFallbackEnabled and self.egoGoalProgressStalled(goalDistance)
        staleCommand = self.egoProgressFallbackEnabled and self.egoLastCommandIsStaleFarFromGoal()
        forceFinalConvergence = self.egoProgressFallbackEnabled and (
            nearFinalTarget or finalConvergenceTimedOut or progressStalled or staleCommand
        )

        if not self.platformFinalHoverActive:
            if forceFinalConvergence:
                self.egoFinalConvergenceForced = True
                now = self.getTimeNow()
                if now - self.egoLastProgressFallbackPrintTime >= 1.0:
                    if nearFinalTarget:
                        print('Near final target; converging final target instead of following EGO local command')
                    elif finalConvergenceTimedOut:
                        print('Final convergence time elapsed; converging final target instead of following EGO local command')
                    elif progressStalled:
                        print('EGO goal progress stalled; converging final target instead of following EGO local command')
                    elif staleCommand:
                        print('EGO command stale far from goal; converging final target instead of following EGO local command')
                    else:
                        print('EGO final convergence forced; converging final target instead of following EGO local command')
                    self.egoLastProgressFallbackPrintTime = now
                self.convergeToEgoGoalPoint(useObstacleGuard=not nearFinalTarget)
            elif commandFresh:
                cmd = self.egoLastCommand
                posENU = point2Array(cmd.position)
                velENU = point2Array(cmd.velocity)
                accENU = point2Array(cmd.acceleration)
                if self.egoCommandIsUsable(posENU, velENU):
                    if self.plannerBackend == 'egov2' and np.all(np.isfinite(accENU)):
                        yawRadENU = float(cmd.yaw) if np.isfinite(cmd.yaw) else self.yawRadENU
                        self.u = np.array(velENU, dtype=float)
                        mavrosVelENU = self.egoV2MavrosVelocityENU(velENU)
                        self.egoV2MavrosVelocityCommandENU = np.array(mavrosVelENU, dtype=float)
                        self.me.trajectoryENUControl(posENU, mavrosVelENU, accENU, yawRadENU)
                        print(
                            f'EGOv2 trajectory pos = {arrayString(posENU)}, '
                            f'vel = {arrayString(velENU)}, '
                            f'px4_vel = {arrayString(mavrosVelENU)}, '
                            f'acc = {arrayString(accENU)}'
                        )
                    elif self.plannerBackend != 'egov2':
                        controlVelENU = self.egoCommandVelocity(posENU, velENU)
                        yawRadENU = yawRadNED2ENU(cmd.yaw)
                        controlVelENU = self.sendBoundedVelocityENUControl(
                            controlVelENU,
                            yawRadENU,
                            useObstacleGuard=True
                        )
                        print(f'EGO target pos = {arrayString(posENU)}, vel cmd = {arrayString(controlVelENU)}')
                    else:
                        print('Rejected EGOv2 command: non-finite acceleration')
                        self.holdCurrentPosition('rejected EGOv2 command')
                else:
                    print('Rejected EGO command; holding current position and waiting for replanning')
                    self.holdCurrentPosition('rejected EGO command')
            elif noCommandFallback:
                now = self.getTimeNow()
                if now - self.egoLastNoCommandFallbackPrintTime >= 1.0:
                    print('No fresh EGO PositionCommand; holding and waiting for replanning')
                    self.egoLastNoCommandFallbackPrintTime = now
                if self.plannerBackend == 'egov2':
                    holdPointENU = (
                        point2Array(self.egoLastCommand.position)
                        if self.egoLastCommand is not None
                        else np.array(self.preparePointENU, dtype=float)
                    )
                    self.u = np.zeros(3)
                    self.me.positionENUControl(holdPointENU, self.yawRadENU)
                else:
                    self.guardedVelocityToPointENUControl(self.me.mePositionENU, self.yawRadENU)
            else:
                print('Waiting for EGO planner PositionCommand; hovering at prepare point')
                self.guardedVelocityToPointENUControl(self.preparePointENU, self.yawRadENU)

        if self.egoGoalPointENU is not None:
            if self.egoGoalIsSettled():
                xyError, zError, speed = self.egoGoalErrorComponents()
                now = self.getTimeNow()
                if self.egoGoalSettledStartTime is None:
                    self.egoGoalSettledStartTime = now
                self.holdEgoGoalPoint()
                settleDuration = now - self.egoGoalSettledStartTime
                if settleDuration >= self.egoGoalSettleTime:
                    if self.switchToPlatformFinalHoverIfNeeded():
                        self.holdEgoGoalPoint()
                        return
                    self.finishState()
                    self.egoGoalReached = True
                    print(
                        f'Final target settled, holding = {arrayString(self.egoGoalPointENU)}, '
                        f'xy err = {xyError:.2f}, z err = {zError:.2f}, speed = {speed:.2f}, '
                        f'settled = {settleDuration:.2f}s'
                    )
                    return
                print(
                    f'Final target inside tolerance; settling: '
                    f'xy err = {xyError:.2f}, z err = {zError:.2f}, speed = {speed:.2f}, '
                    f'settled = {settleDuration:.2f}s'
                )
                return
            else:
                self.egoGoalSettledStartTime = None
                xyError, zError, speed = self.egoGoalErrorComponents()
                if (
                    not self.platformFinalHoverActive and
                    xyError <= self.platformFinalHoverSwitchXYTolerance() and
                    self.switchToPlatformFinalHoverIfNeeded()
                ):
                    self.holdEgoGoalPoint()
                    print(
                        f'Near platform xy; switched to final hover target: '
                        f'xy err = {xyError:.2f}, z err = {zError:.2f}, speed = {speed:.2f}'
                    )
                    return
                if self.platformFinalHoverActive:
                    self.holdEgoGoalPoint()
                    print(
                        f'Holding final hover target: '
                        f'xy err = {xyError:.2f}, z err = {zError:.2f}, speed = {speed:.2f}'
                    )
                    return
                if xyError <= self.egoGoalXYTolerance:
                    self.convergeToEgoGoalPoint()
                    print(
                        f'Near final xy; velocity-converging final target: '
                        f'xy err = {xyError:.2f}, z err = {zError:.2f}, speed = {speed:.2f}'
                    )
                    return
    def name2positionENU(self, name):
        num = self.name2Number(name)
        position = self.communicator.othersInfo[num]['position']
        positionENU = np.array([position.x, position.y, position.z])
        return positionENU

    def name2velocityENU(self, name):
        num = self.name2Number(name)
        velocity = self.communicator.othersInfo[num]['velocity']
        velocityENU = np.array([velocity.x, velocity.y, velocity.z])
        return velocityENU

    def stepBack(self):
        backVelENU = self.me.kp * (self.takeoffPointENU - self.me.mePositionENU)
        self.u = backVelENU
        self.me.velocityENUControl(backVelENU, self.yawRadENU)
        print(f'Back velocity command = {arrayString(backVelENU)}')
        if self.me.nearPositionENU(self.takeoffPointENU):
            self.finishState()
            self.toStepLand()

    def platformLandingComplete(self):
        if not (self.platformLandOnTop and self.platformLandingTargetZ is not None):
            return self.me.belowHeight(height=0.2)

        targetZ = float(self.platformLandingTargetZ)
        xyError = 0.0
        if self.egoGoalPointENU is not None:
            xyError = float(np.linalg.norm(self.me.mePositionENU[:2] - self.egoGoalPointENU[:2]))

        heightError = abs(self.me.mePositionENU[2] - targetZ)
        heightOk = heightError <= self.platformLandingHeightTolerance
        xyOk = xyError <= self.platformLandingXYTolerance
        speedOk = np.linalg.norm(self.me.meVelocityENU) <= self.platformLandingSpeedTolerance
        timeoutOk = self.stateTime >= self.platformLandingTimeout
        disarmed = not self.me.isArmed()

        now = self.getTimeNow()
        if heightOk and xyOk and speedOk:
            if self.platformLandingSettleStartTime is None:
                self.platformLandingSettleStartTime = now
        else:
            self.platformLandingSettleStartTime = None

        settleTime = 0.0
        if self.platformLandingSettleStartTime is not None:
            settleTime = now - self.platformLandingSettleStartTime

        print(
            f'Platform landing check: xy err = {xyError:.2f}, '
            f'z = {self.me.mePositionENU[2]:.2f}, target z = {targetZ:.2f}, '
            f'z err = {heightError:.2f}, speed = {np.linalg.norm(self.me.meVelocityENU):.2f}, '
            f'settled = {settleTime:.2f}s, armed = {self.me.isArmed()}'
        )

        if disarmed and heightOk and xyOk:
            return True
        if settleTime >= self.platformLandingMinimumTime:
            if self.me.isArmed():
                self.me.disarm()
                print('Platform touchdown reached; disarm requested')
                return False
            return True
        if timeoutOk and heightOk and xyOk and speedOk:
            if self.me.isArmed():
                self.me.disarm()
                print('Platform landing timeout reached with stable pose; disarm requested')
                return False
            return True
        return False

    def platformLandingTargetPointENU(self):
        if not (self.platformLandOnTop and self.platformLandingTargetZ is not None):
            return None
        if self.egoGoalPointENU is not None:
            return np.array([
                self.egoGoalPointENU[0],
                self.egoGoalPointENU[1],
                float(self.platformLandingTargetZ),
            ])
        platformCenter = self.platformData.get('centerENU')
        if platformCenter is not None and len(platformCenter) >= 2:
            return np.array([
                float(platformCenter[0]),
                float(platformCenter[1]),
                float(self.platformLandingTargetZ),
            ])
        return None

    def stepPlatformLand(self):
        targetENU = self.platformLandingTargetPointENU()
        if targetENU is None:
            self.holdCurrentPosition('missing platform landing target')
            return

        if self.me.isArmed() and self.me.meState.mode != 'OFFBOARD':
            self.me.intoOffboardMode()

        posErrorENU = targetENU - self.me.mePositionENU
        velENU = np.zeros(3)
        velENU[:2] = self.platformLandingXYGain * posErrorENU[:2]
        velENU[2] = self.platformLandingZGain * posErrorENU[2]
        velENU -= self.platformLandingVelocityDamping * self.me.meVelocityENU
        velENU = self.clipVelocityENU(
            velENU,
            maxSpeed=self.platformLandingMaxSpeed,
            maxVerticalSpeed=self.platformLandingMaxVerticalSpeed
        )

        self.u = velENU
        self.me.velocityENUControl(velENU, self.yawRadENU)
        print(
            f'Platform landing velocity cmd = {arrayString(velENU)}, '
            f'target = {arrayString(targetENU)}'
        )

        if self.platformLandingComplete():
            self.finishState()
            self.toStepEnd()

    def stepLand(self):
        if self.platformLandOnTop and self.platformLandingTargetZ is not None:
            self.stepPlatformLand()
            return

        now = self.getTimeNow()
        shouldSendLand = (
            not self.landCommandSent or
            (self.me.isArmed() and now - self.lastLandCommandTime >= self.platformLandRetryPeriod)
        )
        if shouldSendLand:
            self.me.land()
            self.landCommandSent = True
            self.lastLandCommandTime = now
            print('PX4 land command sent')
        if self.platformLandingComplete():
            self.finishState()
            self.toStepEnd()

    def controlStateMachine(self):
        if self.state == State.INIT:
            self.stepInit()
        elif self.state == State.TAKEOFF:
            self.stepTakeoff()
        elif self.state == State.THROTTLE_TEST:
            self.stepThrottleTest()
        elif self.state == State.PREPARE:
            self.stepPrepare()
        elif self.state == State.GUIDANCE:
            self.stepGuidance()
        elif self.state == State.BACK:
            self.stepBack()
        elif self.state == State.LAND:
            self.stepLand()
        elif self.state == State.END:
            exit(0)        
        elif self.state == State.ATTITUDE_BIAS_TEST:
            self.stepAttitudeBiasTest()

    def safetyModule(self):
        for uav_name, info in self.communicator.othersInfo.items():
            if self.me.nearPositionENU(point2Array(info['position']), tol=self.safetyDistanceBetween):
                print(f'Safety module: too close to {uav_name}, quit...')
                return False
        if self.me.aboveHeight(self.safetyMaxHeight):
            print(f'Safety module: too high, quit...')
            return False
        if self.me.belowHeight(self.safetyMinHeight):
            print(f'Safety module: too low, quit...')
            return False
        if self.me.mePositionENU[0] < self.safetyXMin or self.me.mePositionENU[0] > self.safetyXMax:
            print(f'Safety module: x ({self.me.mePositionENU[0]:.2f}) is out of range ({self.safetyXMin:.2f}, {self.safetyXMax:.2f}), quit...')
            return False
        if self.me.mePositionENU[1] < self.safetyYMin or self.me.mePositionENU[1] > self.safetyYMax:
            print(f'Safety module: y ({self.me.mePositionENU[1]:.2f}) is out of range ({self.safetyYMin:.2f}, {self.safetyYMax:.2f}), quit...')
            return False
        return True

    def print(self):
        now = self.getTimeNow()
        if now - self.lastStatusPrintTime < self.statusPrintPeriod:
            return
        self.lastStatusPrintTime = now

        print('-' * 20)
        print(f'UAV #{self.me.name} / {self.number2Name(self.number, fullName=True)} : state {self.state.name}')
        print(f'Total time: {self.taskTime:.2f}, state time: {self.stateTime:.2f}, self.t: {self.t:.2f}')
        print((RED if self.hz < 0.8 / self.tStep else GREEN) + f"Current frequency: {self.hz} Hz" + RESET)
        print(f'Armed: {"YES" if self.me.isArmed() else "NO"}')
        self.me.printMe()
        print(self.message)

        for uav_number, info in self.communicator.othersInfo.items():
            state = info['state']
            stateFinished = info['stateFinished']
            position = info['position']
            velocity = info['velocity']
            if state is not None and stateFinished is not None and position is not None and velocity is not None:
                print(f'UAV #{uav_number}: state {state}, finished {stateFinished}, pos {pointString(position)}, vel {pointString(velocity)}')
            else:
                print(f'UAV #{uav_number}: No data')

    def updateFrequency(self):
        nowTime = time.time()
        
        self.timestamps.append(nowTime)
        
        while self.timestamps and self.timestamps[0] < nowTime - 1.0:
            self.timestamps.popleft()
        
        self.hz = len(self.timestamps)

    def run(self):
        self.me.sendHeartbeat()
        while self.state != State.END and not rospy.is_shutdown():
            tic = time.time()

            self.applyGazeboTruthStateForFeedback()
            self.updateFrequency()

            self.taskTime = time.time() - self.taskStartTime
            self.stateTime = time.time() - self.stateStartTime

            self.print()

            self.publishEgoPlannerInputs()
            self.publishObstacleVisualization()
            self.controlStateMachine()
            self.log()
            self.me.sendHeartbeat()

            toc = time.time()
            self.elapsedTime = toc - tic
            if self.elapsedTime < self.tStep:
                time.sleep(self.tStep - self.elapsedTime)
                self.t += self.tStep
                self.elapsedTime = self.tStep
            else:
                self.t += self.elapsedTime
            now = self.getTimeNow()
            if now - self.lastElapsedPrintTime >= self.statusPrintPeriod:
                print(f'Elapsed time: {self.elapsedTime:.4f}')
                self.lastElapsedPrintTime = now

    def saveLog(self):
        self.fileName = os.path.join(self.folderName, f'data_{self.number}.pkl')
        with self.logLock:
            if self.logSaved:
                print(f"Data already saved to {self.fileName}")
                return
            dataSnapshot = list(self.data)
            self.logSaved = True

        try:
            with open(self.fileName, "wb") as file:
                pickle.dump({
                    'sceneConfig': self.config,
                    'config': self.config,
                    'data': dataSnapshot,
                    'selfnumber': self.number,
                    'params': self.params
                    }, file)
        except Exception:
            with self.logLock:
                self.logSaved = False
            raise

        print(f"Data saved to {self.fileName}")

    def log(self):
        currentData = {}
        currentData['systemTime'] = copy.copy(datetime.datetime.now())
        currentData['t'] = copy.copy(self.t)
        currentData['taskTime'] = copy.copy(self.taskTime)
        currentData['stateTime'] = copy.copy(self.stateTime)
        currentData['state'] = self.state.name
        currentData['mavrosMode'] = copy.copy(self.me.meState.mode)
        currentData['armed'] = copy.copy(self.me.isArmed())
        currentData['u'] = copy.copy(self.u)
        currentData['mePositionENU'] = copy.copy(self.me.mePositionENU)
        currentData['mePositionNED'] = copy.copy(enu2ned(self.me.mePositionENU))
        currentData['mavrosRawPositionENU'] = copy.copy(self.me.rawPositionENU)
        currentData['mavrosPositionENU'] = copy.copy(self.me.configuredMavrosPositionENU())
        currentData['configuredLocalOffsetENU'] = copy.copy(self.me.configuredLocalOffsetENU)
        currentData['activeLocalOffsetENU'] = copy.copy(self.me.localOffsetENU)
        loggedGazeboPositionENU = self.gazeboPositionENU
        loggedGazeboVelocityENU = self.gazeboVelocityENU
        loggedGazeboQuaternionENU = self.gazeboQuaternionENU
        loggedGazeboOdomTime = self.gazeboOdomTime
        if self.me.externalWorldStateEnabled and self.gazeboAppliedOdomState is not None:
            (
                loggedGazeboPositionENU,
                loggedGazeboVelocityENU,
                loggedGazeboQuaternionENU,
                loggedGazeboOdomTime,
            ) = self.gazeboAppliedOdomState
        if loggedGazeboPositionENU is not None:
            currentData['gazeboPositionENU'] = copy.copy(loggedGazeboPositionENU)
            currentData['gazeboMePositionErrorENU'] = copy.copy(
                loggedGazeboPositionENU - self.me.mePositionENU
            )
            currentData['gazeboMavrosPositionErrorENU'] = copy.copy(
                loggedGazeboPositionENU - self.me.configuredMavrosPositionENU()
            )
            if self.gazeboPoseTime is not None:
                currentData['gazeboPoseAge'] = self.getTimeNow() - self.gazeboPoseTime
        if loggedGazeboQuaternionENU is not None:
            currentData['gazeboQuaternionENU'] = copy.copy(loggedGazeboQuaternionENU)
        if loggedGazeboVelocityENU is not None:
            currentData['gazeboVelocityENU'] = copy.copy(loggedGazeboVelocityENU)
            if loggedGazeboOdomTime is not None:
                currentData['gazeboOdomAge'] = self.getTimeNow() - loggedGazeboOdomTime
        currentData['meVelocity'] = copy.copy(self.me.meVelocityENU)
        currentData['meVelocityNorm'] = copy.copy(np.linalg.norm(self.me.meVelocityENU))
        currentData['meAccelerationENU'] = copy.copy(self.me.meAccelerationENU)
        currentData['safetyViolation'] = self.safetyBoundaryViolation(margin=0.0)
        currentData['safetyWarning'] = self.safetyBoundaryViolation(margin=self.egoSafetyMargin)
        if self.egoPlannerEnabled:
            currentData['plannerBackend'] = copy.copy(self.plannerBackend)
            currentData['rvizEnabled'] = self.rvizEnabled
            currentData['egoMovingObstacleAvoidanceMode'] = copy.copy(self.egoMovingObstacleAvoidanceMode)
            currentData['egoGoalReached'] = copy.copy(self.egoGoalReached)
            currentData['egoCloudPointCount'] = copy.copy(self.egoLastCloudPointCount)
            currentData['egoCloudMovingObstacleCount'] = copy.copy(self.egoLastCloudMovingObstacleCount)
            currentData['egoCloudMovingObstacleIncludedCount'] = copy.copy(
                self.egoLastCloudMovingObstacleIncludedCount
            )
            currentData['egoMovingObstaclePredictionEnabled'] = copy.copy(self.egoLastCloudPredictionEnabled)
            currentData['egoMovingObstaclePointCloudEnabled'] = copy.copy(
                self.egoMovingObstaclePointCloudEnabled
            )
            currentData['egoMovingObstacleTimeAwareCostEnabled'] = copy.copy(
                self.egoMovingObstacleTimeAwareCostEnabled
            )
            if self.plannerBackend == 'egov2':
                currentData['mavrosRawVelocityENU'] = copy.copy(self.me.rawVelocityENU)
                currentData['egoV2VelocityFrameCorrectionENU'] = copy.copy(
                    self.egoV2VelocityFrameCorrectionENU
                )
                currentData['egoV2MavrosVelocityCommandENU'] = copy.copy(
                    self.egoV2MavrosVelocityCommandENU
                )
            if self.egoLastCloudPublishWallTime is not None:
                currentData['egoLastCloudPublishAge'] = self.getTimeNow() - self.egoLastCloudPublishWallTime
            currentData['platformFinalHoverActive'] = copy.copy(self.platformFinalHoverActive)
            currentData['egoObstacleEscapeActive'] = copy.copy(self.egoObstacleEscapeActive)
            currentData['egoObstacleEscapeReason'] = copy.copy(self.egoObstacleEscapeReason)
            currentData['egoObstacleEscapeObstacleName'] = copy.copy(self.egoObstacleEscapeObstacleName)
            currentData['egoObstacleEscapeObstacleType'] = copy.copy(self.egoObstacleEscapeObstacleType)
            currentData['egoObstacleEscapeEntryMargin'] = copy.copy(self.egoObstacleEscapeEntryMargin)
            currentData['egoObstacleEscapeTangentSign'] = copy.copy(self.egoObstacleEscapeTangentSign)
            currentData['egoLastObstaclePointMargin'] = copy.copy(self.egoLastObstaclePointMargin)
            currentData['egoLastObstacleVelocityMargin'] = copy.copy(self.egoLastObstacleVelocityMargin)
            currentData['egoLastObstacleName'] = copy.copy(self.egoLastObstacleName)
            currentData['egoLastObstacleType'] = copy.copy(self.egoLastObstacleType)
            if self.egoObstacleEscapeTargetENU is not None:
                currentData['egoObstacleEscapeTargetENU'] = copy.copy(self.egoObstacleEscapeTargetENU)
            finalHoverPointENU = self.platformFinalHoverPointENU()
            if finalHoverPointENU is not None:
                currentData['platformFinalHoverPointENU'] = copy.copy(finalHoverPointENU)
            if self.egoGoalPointENU is not None:
                currentData['egoGoalPointENU'] = copy.copy(self.egoGoalPointENU)
                currentData['egoControlPointENU'] = copy.copy(self.egoControlPointENU())
                xyError, zError, speed = self.egoGoalErrorComponents()
                currentData['egoGoalXYError'] = xyError
                currentData['egoGoalZError'] = zError
                currentData['egoGoalSpeed'] = speed
            if self.egoLastCommand is not None:
                currentData['egoLastCommandPositionENU'] = point2Array(self.egoLastCommand.position)
                currentData['egoLastCommandVelocityENU'] = point2Array(self.egoLastCommand.velocity)
            if self.egoLastCommandTime is not None:
                currentData['egoCommandAge'] = self.getTimeNow() - self.egoLastCommandTime
        if self.movingObstacleData:
            currentData['movingObstacleStates'] = [
                copy.deepcopy(obstacle)
                for obstacle in self.currentObstacleStates()
                if obstacle.get('type') == 'moving'
            ]
        with self.logLock:
            if not self.logSaved:
                self.data.append(currentData)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--number', type=int, default=0, help='UAV number')
    parser.add_argument('--scene', type=str, default='scene1', help='Scene name')
    parser.add_argument('--prefix', type=str, default=None, help='Run output directory prefix')
    parser.add_argument('--planner-backend', default='ego', choices=['ego', 'egov2'], help='Planner backend')
    parser.add_argument('--enable-rviz', action='store_true', help='Publish SingleRun RViz visualization markers')
    parser.add_argument('--takeoff', help='really takeoff or not', action='store_true')
    args = parser.parse_args()
    sr = SingleRun(**vars(args))
    sr.run()
    sr.saveLog()
    rospy.signal_shutdown('Shutting down')
    sr.spinThread.join()


if __name__ == '__main__':
    main()
