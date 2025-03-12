package org.team9432.frc2025.robot

import choreo.Choreo
import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
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
import org.team9432.frc2025.lib.dashboard.AutoSelector
import org.team9432.frc2025.lib.util.*
import org.team9432.frc2025.robot.FieldConstants.Reef.Branch
import org.team9432.frc2025.robot.ScoringState.CoralScoringTarget
import org.team9432.frc2025.robot.commands.drive.DriveToPose
import org.team9432.frc2025.robot.commands.drive.DrivetrainSimpleFeedforward
import org.team9432.frc2025.robot.commands.drive.WheelRadiusCharacterization
import org.team9432.frc2025.robot.led.LEDState
import org.team9432.frc2025.robot.led.LEDStrip
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
import org.team9432.frc2025.robot.vision.*

class Robot : LoggedRobot() {
    private val driver = CommandXboxController(0)
    private val operator = CommandXboxController(1)
    private val switches = DriverstationSwitches(2)

    private val drive: Drive
    private val superstructure: Superstructure
    private val rollers: Rollers
    private val climber: Climber
    private val scoringState = ScoringState()

    private val leds = LEDStrip(RobotMap.LED_PORT, length = 46)

    private val cameras: Set<Camera>
    private val localizer = Localizer()
    private var simUpdateCall: (() -> Unit)? = null
    private val robotPosition = RobotPosition(localizer)

    private val autoCommands: Auto

    init {
        SignalLogger.start()

        loggerInit()

        val odometryThread = OdometryThread()

        // Run this a few times now so it isn't slow at the start of auto
        for (i in 0..25) {
            Choreo.loadTrajectory("Test Path")
        }

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

        autoCommands = Auto(robotPosition, localizer, drive, superstructure, rollers, scoringState)

        bindButtons()

        PortForwarder.add(5800, "10.94.32.11", 5800)
        PortForwarder.add(5800, "10.94.32.12", 5800)
        PortForwarder.add(5800, "photonvision.local", 5800)

        DriverStation.silenceJoystickConnectionWarning(true)
    }

    private fun bindButtons() {
        superstructure.coastOverride = { switches.one.asBoolean }
        drive.coastOverride = { switches.one.asBoolean }

        val disableAutoAlign = switches.three
        val disableHPAlign = switches.four

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

        driver.x().onTrue(Commands.runOnce({ rollers.clearCoral() }))

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

        val autoAlignForScoringCoral =
            DriveToPose(
                drive,
                localizer,
                {
                    val branch =
                        scoringState.autoBranchTarget
                            ?: robotPosition.nearestReefAlignBranch(
                                localizer.estimatedPose.transformBySpeeds(localizer.robotVelocity, 0.1)
                            )
                    val target = robotPosition.getActiveBranchAlignPose(branch)

                    if (
                        (scoringState.coralTarget == CoralScoringTarget.L1 &&
                            superstructure.currentState != SuperstructureState.PREPARE_L1) ||
                            (scoringState.coralTarget == CoralScoringTarget.L4 &&
                                superstructure.currentState != SuperstructureState.PREPARE_L4)
                    ) {
                        // Wait to drive all the way until arm is in position
                        target.transformBy(Transform2d(-0.25, 0.0, Rotation2d.kZero))
                    } else {
                        target
                    }
                },
                {
                    localizer.getTxTyPose(
                        (scoringState.autoBranchTarget ?: robotPosition.nearestReefAlignBranch()).getTag()
                    ) ?: localizer.estimatedPose
                },
                joystickDriveController,
            )

        RobotModeTriggers.autonomous()
            .and((!rollers.hasCoralTrigger))
            .and { scoringState.autoCoralStationPose != null }
            .whileTrue(autoCommands.autoAlignForStationPickup)

        (driver.rightBumper().or(RobotModeTriggers.autonomous()))
            .and(rollers.hasCoralTrigger.or { isAutonomousEnabled && scoringState.autoCoralStationPose == null })
            .and(!driver.leftBumper())
            .and(!disableAutoAlign)
            .and { scoringState.teleCoralTarget != CoralScoringTarget.L1 }
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
            .and { scoringState.algaeTarget == ScoringState.AlgaeScoringTarget.NET }
            .and {
                localizer.estimatedPose.applyFlip().let {
                    it.y > FieldConstants.fieldWidth / 2 && it.x > (FieldConstants.fieldLength / 2) - 3.0
                }
            }
            .and(!controllerHasRotationInput)
            .and(!disableAutoAlign)
            .whileTrue(drive.runVelocity({ netRotationAlign.calculate() }))

        val processorRotationAlign =
            JoystickAimAtAngleController(
                joystickDriveController,
                {
                    if (localizer.estimatedPose.x > FieldConstants.fieldLength / 2) Rotation2d.kCCW_90deg
                    else Rotation2d.kCW_90deg
                },
                localizer,
            )

        rollers.hasAlgaeTrigger
            .and { scoringState.algaeTarget == ScoringState.AlgaeScoringTarget.PROCESSOR }
            .and {
                localizer.estimatedPose.applyFlip().let {
                    // Along the right wall or close to the opponent processor
                    (it.y < 3.0 && it.x < (FieldConstants.fieldLength / 2)) ||
                        it.distanceTo(FieldConstants.Processor.centerFace.flip()) < 3.0
                }
            }
            .and(!controllerHasRotationInput)
            .and(!disableAutoAlign)
            .whileTrue(drive.runVelocity({ processorRotationAlign.calculate() }))

        val readyToScoreCoral =
            Trigger {
                    val teleopGood =
                        (scoringState.coralTarget == CoralScoringTarget.L3 ||
                            scoringState.coralTarget == CoralScoringTarget.L2) &&
                            robotPosition.withinCoralScoringTolerance.asBoolean

                    val autoGood = robotPosition.withinCoralScoringTolerance.asBoolean

                    teleopGood || autoGood
                }
                .debounce(0.1)

        (driver
                .rightBumper()
                .or(Trigger { localizer.estimatedPose.distanceTo(FieldConstants.Reef.center.applyFlip()) < 2.5 }))
            .and(rollers.hasCoralTrigger)
            .and(!driver.leftBumper())
            .whileTrue(
                superstructure
                    .runGoal {
                        when (scoringState.coralTarget) {
                            CoralScoringTarget.L1 -> SuperstructureState.PREPARE_L1
                            CoralScoringTarget.L2 -> SuperstructureState.PREPARE_L2
                            CoralScoringTarget.L3 -> SuperstructureState.PREPARE_L3
                            CoralScoringTarget.L4 -> SuperstructureState.PREPARE_L4
                        }
                    }
                    .alongWith(Commands.sequence())
            )

        ((driver.a().or(readyToScoreCoral)).and(superstructure::atGoal).and {
                superstructure.currentState.isCoralScoring
            })
            //            .let { it.or { isAutonomousEnabled && it.debounce(.5,
            // Debouncer.DebounceType.kFalling).asBoolean } }
            .whileTrue(rollers.runGoal { rollers.getScoringStateForTarget(scoringState.coralTarget) })

        driver
            .rightBumper()
            .whileTrue(
                (superstructure
                        .runGoal {
                            when (scoringState.algaeTarget) {
                                ScoringState.AlgaeScoringTarget.PROCESSOR -> SuperstructureState.PROCESSOR
                                ScoringState.AlgaeScoringTarget.NET -> SuperstructureState.PREPARE_NET
                            }
                        }
                        .until(driver.a().and(superstructure::atGoal))
                        .andThen(rollers.runGoal(Rollers.State.SCORE_ALGAE)))
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

        driver
            .b()
            .whileTrue(
                superstructure
                    .runGoal { SuperstructureState.ALGAE_FLOOR }
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

        rollers.defaultCommand =
            rollers.runGoal {
                if (!superstructure.isHomed) {
                    Rollers.State.IDLE
                } else {
                    if (rollers.hasAlgae) {
                        Rollers.State.INTAKE_ALGAE
                    } else if (!rollers.hasCoral && superstructure.currentState == SuperstructureState.STOW) {
                        Rollers.State.INTAKE_CORAL
                    } else {
                        Rollers.State.IDLE
                    }
                }
            }

        superstructure.defaultCommand =
            superstructure.runGoal {
                if (robotPosition.isSafeToUseArm.asBoolean) {
                    if (rollers.hasAlgae) {
                        SuperstructureState.ALGAE_STOW
                    } else if (rollers.hasCoral) {
                        when (scoringState.coralTarget) {
                            CoralScoringTarget.L1 -> SuperstructureState.PREPARE_L1
                            CoralScoringTarget.L2 -> SuperstructureState.PREPARE_L2
                            CoralScoringTarget.L3 -> SuperstructureState.PREPARE_L3
                            CoralScoringTarget.L4 -> SuperstructureState.PREPARE_L4
                        }
                    } else {
                        SuperstructureState.STOW
                    }
                } else {
                    superstructure.goal
                }
            }

        operator
            .x()
            .onTrue(Commands.runOnce({ scoringState.teleCoralTarget = CoralScoringTarget.L1 }).ignoringDisable(true))
        operator
            .a()
            .onTrue(Commands.runOnce({ scoringState.teleCoralTarget = CoralScoringTarget.L2 }).ignoringDisable(true))
        operator
            .b()
            .onTrue(Commands.runOnce({ scoringState.teleCoralTarget = CoralScoringTarget.L3 }).ignoringDisable(true))
        operator
            .y()
            .onTrue(Commands.runOnce({ scoringState.teleCoralTarget = CoralScoringTarget.L4 }).ignoringDisable(true))

        operator
            .povUp()
            .onTrue(
                Commands.runOnce({ scoringState.algaeTarget = ScoringState.AlgaeScoringTarget.NET })
                    .ignoringDisable(true)
            )
        operator
            .povDown()
            .onTrue(
                Commands.runOnce({ scoringState.algaeTarget = ScoringState.AlgaeScoringTarget.PROCESSOR })
                    .ignoringDisable(true)
            )

        driver.back().onTrue(Commands.runOnce({ drive.resetGyro() }))
        driver.start().onTrue(superstructure.homeSystem())

        driver.rightStick().and(Constants.robot::isSim).onTrue(Commands.runOnce({ rollers.simSetHasAlgae(true) }))
        driver.leftStick().and(Constants.robot::isSim).onTrue(Commands.runOnce({ rollers.simSetHasCoral(true) }))

        var climbMode = false
        driver.rightStick().onTrue(Commands.runOnce({ climbMode = !climbMode }))
        driver.povUp().and { climbMode }.whileTrue(climber.runGoal(Climber.Goal.UP))
        driver.povDown().and { climbMode }.whileTrue(climber.runGoal(Climber.Goal.DOWN))
        driver.povRight().and { climbMode }.whileTrue(climber.runGoal(Climber.Goal.CLIMB))

        val coralStationRotationAlign =
            JoystickAimAtAngleController(
                joystickDriveController,
                {
                    if (localizer.estimatedPose.applyFlip().y > FieldConstants.fieldWidth / 2)
                        FieldConstants.CoralStation.LEFT.centerPose.rotation.applyFlip()
                    else FieldConstants.CoralStation.RIGHT.centerPose.rotation.applyFlip()
                },
                localizer,
            )

        drive.defaultCommand =
            drive
                .runVelocity({
                    if (!isTeleopEnabled) {
                        ChassisSpeeds()
                    } else {
                        if (
                            superstructure.currentState == SuperstructureState.STOW &&
                                localizer.estimatedPose.applyFlip().x < FieldConstants.fieldLength / 2 &&
                                localizer.estimatedPose.applyFlip().let { robotPose ->
                                    FieldConstants.CoralStation.entries.any {
                                        robotPose.distanceTo(it.centerPose) < 2.0
                                    }
                                } &&
                                !controllerHasRotationInput.asBoolean &&
                                !disableAutoAlign.asBoolean &&
                                !disableHPAlign.asBoolean
                        ) {
                            coralStationRotationAlign.calculate()
                        } else {
                            joystickDriveController.calculate()
                        }
                    }
                })
                .withName("Drive Default")
    }

    private fun CommandGenericHID.rumbleCommand() =
        Commands.startEnd(
                { hid.setRumble(RumbleType.kBothRumble, 1.0) },
                { hid.setRumble(RumbleType.kBothRumble, 0.0) },
            )
            .asProxy()

    private fun CommandGenericHID.alternatingRumbleCommand(periodSeconds: Double) =
        Commands.repeatingSequence(
                Commands.runOnce({
                    hid.setRumble(RumbleType.kLeftRumble, 1.0)
                    hid.setRumble(RumbleType.kRightRumble, 0.0)
                }),
                Commands.waitSeconds(periodSeconds / 2),
                Commands.runOnce({
                    hid.setRumble(RumbleType.kLeftRumble, 0.0)
                    hid.setRumble(RumbleType.kRightRumble, 1.0)
                }),
                Commands.waitSeconds(periodSeconds / 2),
            )
            .finallyDo { _ -> hid.setRumble(RumbleType.kBothRumble, 0.0) }
            .asProxy()

    private var currentAuto = Commands.none()
    private val autoChoosers =
        List(5) { AutoSelector.DashboardQuestion("Option $it Chooser", "Option $it Question") }.toSet()

    private val autoChooser =
        AutoSelector(autoChoosers) {
                addQuestion("Which Auto?", { currentAuto = it }) {
                    addOption("Do Nothing (Broken)", Commands::none)

                    addOption("Max L4 Left", { autoCommands.maxL4Left() })

                    addOption("Only L2") {
                        addQuestion("Side", { currentAuto = it }) {
                            addOption("Left", { autoCommands.onlyL2(Branch.J) })
                            addOption("Right", { autoCommands.onlyL2(Branch.E) })
                            var branch: Branch? = null
                            addOption("Custom", { branch?.let { autoCommands.onlyL2(it) } }) {
                                addQuestion("Which Branch?", { branch = it }) {
                                    Branch.entries.forEach { addOption(it.name, { it }) }
                                }
                            }
                        }
                    }

                    var characterizationAuto = Commands.none()
                    addOption("Characterization", { characterizationAuto }) {
                        addQuestion("Which routine?", { characterizationAuto = it }) {
                            addOption(
                                "Drive Simple Feedforward Characterization",
                                { DrivetrainSimpleFeedforward(drive) },
                            )
                            addOption(
                                "Drive Wheel Radius Characterization",
                                { WheelRadiusCharacterization(drive, localizer) },
                            )
                            addOption(
                                "Elevator Static Characterization",
                                { superstructure.elevatorStaticCharacterization() },
                            )
                            addOption(
                                "CoralArm Static Characterization",
                                { superstructure.armStaticCharacterization() },
                            )
                        }
                    }
                }
            }
            .also { it.update() }

    override fun autonomousInit() {
        currentAuto.schedule()
    }

    override fun autonomousExit() {
        currentAuto.cancel()
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
        scoringState.clearAutoState()
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
        scoringState.log()

        autoChooser.update()

        LEDState.updateBuffer(leds.buffer)
        leds.displayBuffer()
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
