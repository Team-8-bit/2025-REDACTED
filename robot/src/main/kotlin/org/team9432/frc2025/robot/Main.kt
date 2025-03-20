package org.team9432.frc2025.robot

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.abs
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.photonvision.simulation.VisionSystemSim
import org.team9432.frc2025.lib.AllianceTracker
import org.team9432.frc2025.lib.util.*
import org.team9432.frc2025.robot.RobotState.AlgaeScoringTarget
import org.team9432.frc2025.robot.RobotState.CoralScoringTarget
import org.team9432.frc2025.robot.commands.drive.DriveToPose
import org.team9432.frc2025.robot.subsystems.climber.Climber
import org.team9432.frc2025.robot.subsystems.climber.ClimberIO
import org.team9432.frc2025.robot.subsystems.climber.ClimberIOReal
import org.team9432.frc2025.robot.subsystems.drive.Drive
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.subsystems.drive.ModuleConfig
import org.team9432.frc2025.robot.subsystems.drive.OdometryThread
import org.team9432.frc2025.robot.subsystems.drive.controllers.JoystickAimAtAngleController
import org.team9432.frc2025.robot.subsystems.drive.controllers.JoystickDriveController
import org.team9432.frc2025.robot.subsystems.drive.gyro.GyroIO
import org.team9432.frc2025.robot.subsystems.drive.gyro.GyroIOPigeon2
import org.team9432.frc2025.robot.subsystems.drive.gyro.GyroIOSim
import org.team9432.frc2025.robot.subsystems.drive.module.ModuleIO
import org.team9432.frc2025.robot.subsystems.drive.module.ModuleIOReal
import org.team9432.frc2025.robot.subsystems.drive.module.ModuleIOSim
import org.team9432.frc2025.robot.subsystems.rollers.Rollers
import org.team9432.frc2025.robot.subsystems.rollers.dispenser.Manipulator
import org.team9432.frc2025.robot.subsystems.rollers.dispenser.ManipulatorIO
import org.team9432.frc2025.robot.subsystems.rollers.dispenser.ManipulatorIOReal
import org.team9432.frc2025.robot.subsystems.rollers.funnel.Funnel
import org.team9432.frc2025.robot.subsystems.rollers.funnel.FunnelIO
import org.team9432.frc2025.robot.subsystems.rollers.funnel.FunnelIOReal
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure
import org.team9432.frc2025.robot.subsystems.superstructure.SuperstructureState
import org.team9432.frc2025.robot.subsystems.superstructure.arm.Arm
import org.team9432.frc2025.robot.subsystems.superstructure.arm.ArmIO
import org.team9432.frc2025.robot.subsystems.superstructure.arm.ArmIOReal
import org.team9432.frc2025.robot.subsystems.superstructure.arm.ArmIOSim
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.ElevatorIO
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.ElevatorIOReal
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.ElevatorIOSim
import org.team9432.frc2025.robot.util.*
import org.team9432.frc2025.robot.vision.*

class Robot : LoggedRobot() {
    private val driver = CommandXboxController(0)
    private val operator = CommandXboxController(1)
    private val switches = DriverstationSwitches(2)

    private val drive: Drive
    private val superstructure: Superstructure
    private val rollers: Rollers
    private val climber: Climber
    private val robotState = RobotState()

    private val cameras: Set<Camera>
    private val localizer = Localizer()
    private var simUpdateCall: (() -> Unit)? = null
    private val robotPosition = RobotPosition(localizer)

    private val autoCommands: Auto
    private val autoChooser: AutoChooser
    private val seesDisabledTagDebouncer = Debouncer(0.5, Debouncer.DebounceType.kFalling)

    init {
        LEDState.codeLoading = true
        LEDState.updateBuffer(LEDStrip.buffer)
        LEDStrip.displayBuffer()

        SignalLogger.start()

        loggerInit()

        val odometryThread = OdometryThread()

        // No need to instantiate subsystems if the robot is running in replay
        if (!Constants.mode.isReplay) {
            // Switch based on the selected robot
            when (Constants.robot) {
                Constants.RobotType.COMP -> {
                    drive =
                        Drive(
                            GyroIOPigeon2(odometryThread),
                            ModuleIOReal(ModuleConfig.FRONT_LEFT, odometryThread),
                            ModuleIOReal(ModuleConfig.FRONT_RIGHT, odometryThread),
                            ModuleIOReal(ModuleConfig.BACK_LEFT, odometryThread),
                            ModuleIOReal(ModuleConfig.BACK_RIGHT, odometryThread),
                            odometryThread,
                            localizer,
                        )

                    superstructure = Superstructure(Elevator(ElevatorIOReal()), Arm(ArmIOReal()))
                    rollers = Rollers(Funnel(FunnelIOReal()), Manipulator(ManipulatorIOReal()))
                    climber = Climber(ClimberIOReal())

                    cameras =
                        setOf(
                            Camera(
                                CameraIOPhotonVision(VisionConstants.CameraConstants.FRONT_LEFT, localizer::rotation),
                                VisionConstants.CameraConstants.FRONT_LEFT,
                                localizer,
                            ),
                            Camera(
                                CameraIOPhotonVision(VisionConstants.CameraConstants.FRONT_RIGHT, localizer::rotation),
                                VisionConstants.CameraConstants.FRONT_RIGHT,
                                localizer,
                            ),
                        )
                }

                Constants.RobotType.SIM -> {
                    val swerveSim =
                        SwerveDriveSimulation(
                            DriveTrainSimulationConfig.Default()
                                .withSwerveModule {
                                    SwerveModuleSimulation(
                                        SwerveModuleSimulationConfig(
                                            /* driveMotorModel = */ DCMotor.getKrakenX60Foc(1),
                                            /* steerMotorModel = */ DCMotor.getKrakenX60Foc(1),
                                            /* driveGearRatio = */ DrivetrainConstants.DRIVE_RATIO,
                                            /* steerGearRatio = */ DrivetrainConstants.STEER_RATIO,
                                            /* driveFrictionVoltage = */ Volts.of(
                                                0.1
                                            ), // Just the value used in the maplesim MK4i default
                                            /* steerFrictionVoltage = */ Volts.of(
                                                0.2
                                            ), // Just the value used in the maplesim MK4i default
                                            /* wheelRadius = */ Meters.of(DrivetrainConstants.WHEEL_RADIUS),
                                            /* steerRotationalInertia = */ KilogramSquareMeters.of(
                                                0.03
                                            ), // Just the value used in the maplesim MK4i default
                                            /* wheelsCoefficientOfFriction = */ COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                                        )
                                    )
                                }
                                .withGyro(COTS.ofPigeon2())
                                .withTrackLengthTrackWidth(Inches.of(24.25), Inches.of(24.25))
                                .withBumperSize(
                                    Meters.of(DrivetrainConstants.BUMPER_LENGTH),
                                    Meters.of(DrivetrainConstants.BUMPER_LENGTH),
                                )
                                .withRobotMass(Pounds.of(135.0)),
                            /* initialPoseOnField = */ Pose2d(7.0, 6.175, Rotation2d.fromDegrees(225.0)),
                        )

                    val gyroIO = GyroIOSim(swerveSim.gyroSimulation, odometryThread)

                    val (frontLeft, frontRight, backLeft, backRight) = swerveSim.modules

                    drive =
                        Drive(
                            gyroIO,
                            ModuleIOSim(frontLeft, ModuleConfig.FRONT_LEFT, odometryThread),
                            ModuleIOSim(frontRight, ModuleConfig.FRONT_RIGHT, odometryThread),
                            ModuleIOSim(backLeft, ModuleConfig.BACK_LEFT, odometryThread),
                            ModuleIOSim(backRight, ModuleConfig.BACK_RIGHT, odometryThread),
                            odometryThread,
                            localizer,
                        )

                    SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim)

                    localizer.setSimulationPoseSupplier { swerveSim.simulatedDriveTrainPose }

                    val visionSim = VisionSystemSim("main").apply { addAprilTags(VisionConstants.aprilTagLayout) }

                    cameras =
                        setOf(
                            Camera(
                                CameraIOPhotonVisionSim(
                                    VisionConstants.CameraConstants.FRONT_LEFT,
                                    visionSim,
                                    localizer::rotation,
                                ),
                                VisionConstants.CameraConstants.FRONT_LEFT,
                                localizer,
                            ),
                            Camera(
                                CameraIOPhotonVisionSim(
                                    VisionConstants.CameraConstants.FRONT_RIGHT,
                                    visionSim,
                                    localizer::rotation,
                                ),
                                VisionConstants.CameraConstants.FRONT_RIGHT,
                                localizer,
                            ),
                        )

                    simUpdateCall = { visionSim.update(swerveSim.simulatedDriveTrainPose) }

                    superstructure = Superstructure(Elevator(ElevatorIOSim()), Arm(ArmIOSim()))
                    rollers = Rollers(Funnel(object : FunnelIO {}), Manipulator(object : ManipulatorIO {}))
                    climber = Climber(object : ClimberIO {})
                }
            }
        } else {
            // No-op replay implementations
            drive =
                Drive(
                    object : GyroIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    odometryThread,
                    localizer,
                )

            superstructure = Superstructure(Elevator(object : ElevatorIO {}), Arm(object : ArmIO {}))
            rollers = Rollers(Funnel(object : FunnelIO {}), Manipulator(object : ManipulatorIO {}))
            climber = Climber(object : ClimberIO {})

            cameras =
                setOf(
                    Camera(object : CameraIO {}, VisionConstants.CameraConstants.FRONT_LEFT, localizer),
                    Camera(object : CameraIO {}, VisionConstants.CameraConstants.FRONT_RIGHT, localizer),
                )
        }

        if (Constants.mode != Constants.Mode.REPLAY) {
            odometryThread.start()
        }

        autoCommands = Auto(robotPosition, localizer, drive, superstructure, rollers, robotState)
        autoChooser = AutoChooser(autoCommands, localizer, drive, superstructure)

        //        Elevator.Goal.ADAPTIVE_SCORE_L2.overrideSetpointSupplier = {
        //
        // AdaptiveDistanceLookupTable.L2.getElevatorHeight(robotPosition.nearestBranchDistance)
        //        }
        //        Arm.Goal.ADAPTIVE_SCORE_L2.overrideSetpointSupplier = {
        //
        // AdaptiveDistanceLookupTable.L2.getArmAngle(robotPosition.nearestBranchDistance)
        //        }
        //
        //        Elevator.Goal.ADAPTIVE_SCORE_L3.overrideSetpointSupplier = {
        //
        // AdaptiveDistanceLookupTable.L3.getElevatorHeight(robotPosition.nearestBranchDistance)
        //        }
        //        Arm.Goal.ADAPTIVE_SCORE_L3.overrideSetpointSupplier = {
        //
        // AdaptiveDistanceLookupTable.L3.getArmAngle(robotPosition.nearestBranchDistance)
        //        }

        bindButtons()

        PortForwarder.add(5800, "10.94.32.11", 5800)
        PortForwarder.add(5800, "10.94.32.12", 5800)
        PortForwarder.add(5800, "photonvision.local", 5800)

        DriverStation.silenceJoystickConnectionWarning(true)

        LEDState.visionDisconnected = { cameras.any { !it.connected } }
        LEDState.seesDisabledTag = { seesDisabledTagDebouncer.calculate(isDisabled && cameras.any { it.seesAnyTag }) }
        LEDState.displayElevatorHeight = { false }
        LEDState.shouldRunDisplay = { switches.seven.asBoolean }

        LEDState.codeLoading = false
    }

    private fun bindButtons() {
        superstructure.coastOverride = { switches.one.asBoolean && isDisabled }
        drive.coastOverride = { switches.one.asBoolean && isDisabled }

        rollers.disableBeambreak = { switches.eight.asBoolean }

        val disableAutoAlign = switches.three

        val joystickDriveController =
            JoystickDriveController(
                controllerX = { -driver.leftY },
                controllerY = { -driver.leftX },
                controllerR = { driver.leftTriggerAxis - driver.rightTriggerAxis },
                localizer,
            )

        val controllerHasDriveInput = Trigger {
            joystickDriveController.hasInput(xyDeadband = 0.1, rotationalDeadband = 0.1)
        }
        val controllerHasRotationInput = Trigger { joystickDriveController.hasRotationInput(deadband = 0.1) }

        val autoAlignForCollectingAlgae =
            DriveToPose(
                drive,
                localizer,
                {
                    val branch = robotPosition.nearestAlgaePickup(localizer.estimatedPose)
                    val pose = robotPosition.getActiveAlgaeAlignPose(branch)
                    if (rollers.hasAlgae) {
                        // Drive back after pickup
                        pose.transformBy(Transform2d(-0.5, 0.0, Rotation2d.kZero))
                    } else if (
                        !(superstructure.currentState == SuperstructureState.INTAKE_ALGAE_LOW ||
                            superstructure.currentState == SuperstructureState.INTAKE_ALGAE_HIGH)
                    ) {
                        // Wait to lower algae arm
                        pose.transformBy(Transform2d(-0.25, 0.0, Rotation2d.kZero))
                    } else {
                        pose
                    }
                },
                { localizer.getTxTyPose(robotPosition.nearestAlgaePickup().getTag()) ?: localizer.estimatedPose },
            )

        var lastBranch: FieldConstants.Reef.Branch? = null
        val autoAlignForScoringCoral =
            DriveToPose(
                    drive,
                    localizer,
                    {
                        var branch =
                            robotState.autoBranchTarget ?: robotPosition.nearestReefAlignBranch(localizer.estimatedPose)

                        if (branch != lastBranch) {
                            lastBranch = branch
                            robotState.flipBranch = false
                        }

                        if (robotState.flipBranch) {
                            branch = branch.oppositeOnFace
                        }

                        val target = robotPosition.getActiveBranchAlignPose(branch)

                        val isNotReadyForL4 =
                            robotState.coralTarget == CoralScoringTarget.L4 &&
                                superstructure.currentState !in
                                    setOf(SuperstructureState.PREP_L4, SuperstructureState.SCORE_L4)
                        val isNotReadyForL1 =
                            robotState.coralTarget == CoralScoringTarget.L1 &&
                                superstructure.currentState == SuperstructureState.SCORE_L1

                        val armNotReady = isNotReadyForL1 || isNotReadyForL4
                        if (armNotReady) {
                            // Wait to drive all the way until arm is in position
                            target.transformBy(Transform2d(-0.25, 0.0, Rotation2d.kZero))
                        } else {
                            target
                        }
                    },
                    {
                        localizer.getTxTyPose(
                            (robotState.autoBranchTarget ?: robotPosition.nearestReefAlignBranch()).getTag()
                        ) ?: localizer.estimatedPose
                    },
                    joystickDriveController,
                    {
                        MathUtil.clamp(
                            (localizer.estimatedPose.distanceTo(FieldConstants.Reef.center.applyFlip()) -
                                FieldConstants.Reef.maxRadius -
                                (DrivetrainConstants.BUMPER_LENGTH / 2)) * 2.5,
                            2.5,
                            5.0,
                        )
                    },
                )
                .apply { name = "AutoAlignForScoringCoral" }

        val autoAlignForScoringProcessor =
            DriveToPose(
                drive,
                localizer,
                { robotPosition.getActiveProcessorAlignPose() },
                { localizer.estimatedPose },
                joystickDriveController,
            )

        RobotModeTriggers.autonomous()
            .and(!rollers.hasCoralTrigger)
            .and { robotState.autoCoralStationPose != null }
            .whileTrue(autoCommands.autoAlignForStationPickup)

        (driver.rightBumper().or(RobotModeTriggers.autonomous()))
            //            .and({ robotState.autoCoralStationPose == null })
            .and((!rollers.hasAlgaeTrigger).or { isAutonomousEnabled && robotState.autoCoralStationPose == null })
            .and { !superstructure.currentState.isAlgaeScoring }
            .and(!driver.leftBumper())
            .and(!disableAutoAlign)
            .and { robotState.teleCoralTarget != CoralScoringTarget.L1 }
            .whileTrue(autoAlignForScoringCoral)

        driver
            .leftBumper()
            .and(!rollers.hasCoralTrigger)
            .and(!controllerHasDriveInput)
            .and(!disableAutoAlign)
            .whileTrue(autoAlignForCollectingAlgae)

        val netRotationAlign =
            JoystickAimAtAngleController(joystickDriveController, { Rotation2d.kZero.applyFlip() }, localizer)
        rollers.hasAlgaeTrigger
            .and { robotState.algaeTarget == AlgaeScoringTarget.NET }
            .and {
                val blueSidePose = localizer.estimatedPose.applyFlip()
                blueSidePose.y > FieldConstants.fieldWidth / 2 &&
                    blueSidePose.x > (FieldConstants.fieldLength / 2) - 3.0 &&
                    abs(blueSidePose.rotation.degrees) < 70
            }
            .and(!controllerHasRotationInput)
            .and(!disableAutoAlign)
            .whileTrue(drive.runVelocity({ netRotationAlign.calculate() }))

        rollers.hasAlgaeTrigger
            .and { robotState.algaeTarget == AlgaeScoringTarget.PROCESSOR }
            .and(driver.rightBumper())
            .and(!disableAutoAlign)
            .whileTrue(autoAlignForScoringProcessor)

        val withinTolerance =
            robotPosition.withinCoralScoringTolerance
                .debounce(0.05, Debouncer.DebounceType.kRising)
                .debounce(0.5, Debouncer.DebounceType.kFalling)
                .and(RobotModeTriggers.autonomous())

        (driver.a().or(withinTolerance))
            .and { superstructure.goal.isCoralScoring }
            .and(superstructure::atGoal)
            .whileTrue(
                rollers
                    .runGoal { rollers.getScoringStateForTarget(robotState.coralTarget) }
                    .finallyDo { interrupted -> robotState.flipBranch = false }
            )

        driver
            .rightBumper()
            .whileTrue(
                (superstructure
                        .runGoal {
                            when (robotState.algaeTarget) {
                                AlgaeScoringTarget.PROCESSOR -> SuperstructureState.PROCESSOR
                                AlgaeScoringTarget.NET -> SuperstructureState.PREP_NET
                            }
                        }
                        .until(
                            (driver.a().or {
                                    autoAlignForScoringProcessor.withinTolerance(1.5, Units.degreesToRotations(1.5))
                                })
                                .and(superstructure::atGoal)
                        )
                        .andThen(rollers.runGoal(Rollers.State.SCORE_ALGAE)))
                    .asProxy()
                    .onlyIf(rollers.hasAlgaeTrigger)
            )

        driver
            .leftBumper()
            .whileTrue(
                superstructure
                    .runGoal {
                        if (!robotPosition.isSafeToUseArm.asBoolean) {
                            superstructure.goal
                        } else {
                            if (robotPosition.nearestAlgaePickup().isHigh) {
                                SuperstructureState.INTAKE_ALGAE_HIGH
                            } else {
                                SuperstructureState.INTAKE_ALGAE_LOW
                            }
                        }
                    }
                    .alongWith(rollers.runGoal(Rollers.State.INTAKE_ALGAE))
            )

        rollers.hasAlgaeTrigger.onTrue(driver.rumbleCommand().withTimeout(0.5))

        RobotModeTriggers.teleop()
            .and(
                Trigger {
                    val matchTime = Timer.getMatchTime()
                    matchTime < 25 && matchTime > 10
                }
            )
            .whileTrue(driver.alternatingRumbleCommand(0.25))

        val homeSystemCommand = superstructure.homeSystem()

        rollers.defaultCommand =
            rollers.runGoal {
                if (!superstructure.isHomed || homeSystemCommand.isScheduled) {
                    Rollers.State.IDLE
                } else if (rollers.hasAlgae) {
                    Rollers.State.INTAKE_ALGAE
                } else if (!rollers.hasCoral && superstructure.currentState == SuperstructureState.STOW) {
                    Rollers.State.INTAKE_CORAL
                } else if (superstructure.currentState == SuperstructureState.STOW || switches.four.asBoolean) {
                    Rollers.State.IDLE
                } else {
                    Rollers.State.UNJAM_CORAL
                }
            }

        superstructure.defaultCommand =
            superstructure.runGoal {
                val threeToTwo =
                    robotState.coralTarget == CoralScoringTarget.L2 &&
                        superstructure.currentState in
                            setOf(SuperstructureState.SCORE_L3, SuperstructureState.ADAPTIVE_SCORE_L3)
                val twoToThree =
                    robotState.coralTarget == CoralScoringTarget.L3 &&
                        superstructure.currentState in
                            setOf(SuperstructureState.SCORE_L2, SuperstructureState.ADAPTIVE_SCORE_L2)

                val fourToFour =
                    robotState.coralTarget == CoralScoringTarget.L4 &&
                        superstructure.currentState == SuperstructureState.PREP_L4

                val stowToTwoOrThree =
                    robotState.coralTarget in setOf(CoralScoringTarget.L2, CoralScoringTarget.L3) &&
                        superstructure.currentState == SuperstructureState.STOW

                if (
                    robotPosition.isSafeToUseArm.asBoolean || threeToTwo || twoToThree || fourToFour || stowToTwoOrThree
                ) {
                    if (rollers.hasAlgae) {
                        SuperstructureState.ALGAE_STOW
                    } else if (rollers.hasCoral) {
                        when (robotState.coralTarget) {
                            CoralScoringTarget.L1 -> SuperstructureState.SCORE_L1
                            CoralScoringTarget.L2 -> SuperstructureState.ADAPTIVE_SCORE_L2
                            CoralScoringTarget.L3 -> SuperstructureState.ADAPTIVE_SCORE_L3
                            CoralScoringTarget.L4 -> {
                                val shouldFullyExtend =
                                    localizer.estimatedPose.distanceTo(FieldConstants.Reef.center.applyFlip()) -
                                        FieldConstants.Reef.maxRadius -
                                        (DrivetrainConstants.BUMPER_LENGTH / 2) < 1.0 &&
                                        robotPosition.angleFromReef() < 45
                                if (shouldFullyExtend) {
                                    SuperstructureState.SCORE_L4
                                } else {
                                    SuperstructureState.PREP_L4
                                }
                            }
                        }
                    } else {
                        SuperstructureState.STOW
                    }
                } else {
                    superstructure.goal
                }
            }

        val backupButton = driver.back()

        (operator.x())
            .or((driver.povLeft().and(!backupButton)))
            .onTrue(Commands.runOnce({ robotState.teleCoralTarget = CoralScoringTarget.L1 }).ignoringDisable(true))
        (operator.a())
            .or((driver.povDown().and(!backupButton)))
            .onTrue(Commands.runOnce({ robotState.teleCoralTarget = CoralScoringTarget.L2 }).ignoringDisable(true))
        (operator.b())
            .or((driver.povRight().and(!backupButton)))
            .onTrue(Commands.runOnce({ robotState.teleCoralTarget = CoralScoringTarget.L3 }).ignoringDisable(true))
        (operator.y())
            .or((driver.povUp().and(!backupButton)))
            .onTrue(Commands.runOnce({ robotState.teleCoralTarget = CoralScoringTarget.L4 }).ignoringDisable(true))

        (operator.povUp())
            .or((driver.povUp().and(!backupButton)))
            .onTrue(Commands.runOnce({ robotState.algaeTarget = AlgaeScoringTarget.NET }).ignoringDisable(true))
        (operator.povDown())
            .or((driver.povDown().and(!backupButton)))
            .onTrue(Commands.runOnce({ robotState.algaeTarget = AlgaeScoringTarget.PROCESSOR }).ignoringDisable(true))

        driver.start().and(!backupButton).onTrue(homeSystemCommand)
        driver.start().and(backupButton).onTrue(Commands.runOnce(drive::resetGyro))

        driver.x().and(backupButton).onTrue(Commands.runOnce({ rollers.clearCoral() }))
        driver.y().whileTrue(superstructure.runGoal { SuperstructureState.UNJAM_CORAL })
        driver
            .b()
            .and(backupButton)
            .whileTrue(
                superstructure
                    .runGoal { SuperstructureState.ALGAE_FLOOR }
                    .alongWith(rollers.runGoal(Rollers.State.INTAKE_ALGAE))
            )

        driver.rightStick().whileTrue(rollers.runGoal(Rollers.State.UNJAM_CORAL))

        (driver.povUp().and(backupButton)).or(operator.rightBumper()).whileTrue(climber.runGoal(Climber.Goal.UP))
        (driver.povDown().and(backupButton)).or(operator.leftBumper()).whileTrue(climber.runGoal(Climber.Goal.DOWN))
        driver.x().and(!backupButton).whileTrue(climber.runGoal(Climber.Goal.CLIMB))
        driver.b().and(!backupButton).whileTrue(Commands.runOnce({ robotState.flipBranch = true }))

        drive.defaultCommand =
            drive
                .runVelocity({
                    if (!isTeleopEnabled) {
                        ChassisSpeeds()
                    } else {
                        joystickDriveController.calculate()
                    }
                })
                .withName("Drive Default")

        driver.rightStick().and(Constants.robot::isSim).onTrue(Commands.runOnce({ rollers.simSetHasAlgae(true) }))
        driver.leftStick().and(Constants.robot::isSim).onTrue(Commands.runOnce({ rollers.simSetHasCoral(true) }))

        LEDState.isAutoAligning = {
            autoAlignForScoringCoral.running ||
                autoAlignForCollectingAlgae.running ||
                autoAlignForScoringProcessor.running
        }
    }

    override fun autonomousInit() {
        autoChooser.command.schedule()
    }

    override fun autonomousExit() {
        autoChooser.command.cancel()
    }

    private fun loggerInit() {
        Logger.recordMetadata("Robot", Constants.robot.toString())
        Logger.recordMetadata("TuningMode", Constants.TUNING_MODE.toString())
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString())
        Logger.recordMetadata("ProjectName", MAVEN_NAME)
        Logger.recordMetadata("GitSha", GIT_SHA)
        Logger.recordMetadata("GitDate", GIT_DATE)
        Logger.recordMetadata("GitBranch", GIT_BRANCH)
        Logger.recordMetadata("BuildDate", BUILD_DATE)
        Logger.recordMetadata("GitDirty", if (DIRTY == 1) "true" else "false")

        when (Constants.mode) {
            Constants.Mode.REAL -> {
                Logger.addDataReceiver(WPILOGWriter()) // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
                PowerDistribution(1, PowerDistribution.ModuleType.kRev) // Enables power distribution logging
            }

            Constants.Mode.SIM -> {
                Logger.addDataReceiver(NT4Publisher())
                PowerDistribution(1, PowerDistribution.ModuleType.kRev) // Enables power distribution logging
            }

            Constants.Mode.REPLAY -> {
                setUseTiming(false) // Run as fast as possible
                val logPath =
                    LogFileUtil.findReplayLog() // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
                Logger.addDataReceiver(
                    WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay"))
                ) // Save outputs to a new log
            }
        }

        Logger.start() // Start logging! No more data receivers, replay sources, or metadata values may be added.

        // Command Logging
        val commandCounts: MutableMap<String, Int> = HashMap()
        fun logCommand(command: Command, starting: Boolean) {
            val name = command.name
            val count = commandCounts.getOrDefault(name, 0) + (if (starting) 1 else -1)
            commandCounts[name] = count
            Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), starting)
            Logger.recordOutput("CommandsAll/$name", count > 0)
        }

        CommandScheduler.getInstance().run {
            onCommandInitialize { command -> logCommand(command, true) }
            onCommandFinish { command -> logCommand(command, false) }
            onCommandInterrupt { command -> logCommand(command, false) }
        }
    }

    override fun teleopPeriodic() {
        robotState.clearAutoState()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        DriverStation.getAlliance().ifPresent { AllianceTracker.currentAlliance = it }

        // Log CANivore status if running on a real robot
        if (Constants.mode.isReal) {
            val canivoreStatus = RobotMap.drivetrainCanbus.status
            Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.Status.getName())
            Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.BusUtilization)
            Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.BusOffCount)
            Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.TxFullCount)
            Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.REC)
            Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.TEC)
        }

        // Log robot state
        localizer.log()
        robotPosition.outputTelemetry()
        robotState.log()

        autoChooser.update()

        LEDState.updateBuffer(LEDStrip.buffer)
        LEDStrip.displayBuffer()
    }

    override fun simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic()
        simUpdateCall?.invoke()
    }
}

/**
 * Main initialization function. Do not perform any initialization here other than calling `RobotBase.startRobot`. Do
 * not modify this file except to change the object passed to the `startRobot` call.
 *
 * If you change the package of this file, you must also update the `ROBOT_MAIN_CLASS` variable in the gradle build
 * file. Note that this file has a `@file:JvmName` annotation so that its compiled Java class name is "Main" rather than
 * "MainKt". This is to prevent any issues/confusion if this file is ever replaced with a Java class.
 *
 * If you change your main Robot object (name), change the parameter of the `RobotBase.startRobot` call to the new name.
 * (If you use the IDE's Rename Refactoring when renaming the object, it will get changed everywhere including here.)
 */
fun main() {
    RobotBase.startRobot { Robot() }
}
