package org.team9432.frc2025.robot

import choreo.Choreo
import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
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
import org.team9432.frc2025.lib.util.not
import org.team9432.frc2025.robot.commands.drive.DrivetrainSysIdCommands
import org.team9432.frc2025.robot.commands.drive.WheelRadiusCharacterization
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

    private val cameras: Set<Camera>
    private val localizer = Localizer()
    private var simUpdateCall: (() -> Unit)? = null
    private val robotPosition = RobotPosition(localizer)

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
                                            /* wheelRadius = */ Inches.of(DrivetrainConstants.WHEEL_RADIUS_INCHES),
                                            /* steerRotationalInertia = */ KilogramSquareMeters.of(
                                                0.03
                                            ), // Just the value used in the maplesim MK4i default
                                            /* wheelsCoefficientOfFriction = */ COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                                        )
                                    )
                                }
                                .withGyro(COTS.ofPigeon2())
                                .withTrackLengthTrackWidth(Inches.of(24.0), Inches.of(24.0))
                                .withBumperSize(Inches.of(30.0), Inches.of(30.0))
                                .withRobotMass(Pounds.of(120.0)),
                            /* initialPoseOnField = */ Pose2d(3.0, 3.0, Rotation2d()),
                        )

                    val gyroIO = GyroIOSim(swerveSim.gyroSimulation)

                    val (frontLeft, frontRight, backLeft, backRight) = swerveSim.modules

                    drive =
                        Drive(
                            gyroIO,
                            ModuleIOSim(frontLeft),
                            ModuleIOSim(frontRight),
                            ModuleIOSim(backLeft),
                            ModuleIOSim(backRight),
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

        bindButtons()

        PortForwarder.add(5800, "10.94.32.11", 5800)
        PortForwarder.add(5800, "10.94.32.12", 5800)
        PortForwarder.add(5800, "photonvision.local", 5800)

        DriverStation.silenceJoystickConnectionWarning(true)
    }

    private fun bindButtons() {
        superstructure.coastOverride = { switches.one.asBoolean }

        val joystickDriveController =
            JoystickDriveController(
                controllerX = { -driver.leftY },
                controllerY = { -driver.leftX },
                controllerR = { driver.leftTriggerAxis - driver.rightTriggerAxis },
                localizer,
            )

        val alignStraightController =
            JoystickAimAtAngleController(joystickDriveController, { Rotation2d.kZero }, localizer)

        val prepareScoreButton = driver.rightBumper()

        val doublePressIntakeTimer = Timer()

        driver
            .leftBumper()
            .negate()
            .and { !doublePressIntakeTimer.hasElapsed(0.15) }
            .onTrue(rollers.runGoal(Rollers.State.UNJAM_CORAL).withTimeout(0.5))

        driver
            .leftBumper()
            .and(!prepareScoreButton)
            .and(!rollers.hasAlgaeTrigger)
            .onTrue(Commands.runOnce({ doublePressIntakeTimer.restart() }))
            .whileTrue(
                superstructure
                    .runGoal(Superstructure.State.INTAKE_CORAL)
                    .alongWith(rollers.runGoal(Rollers.State.INTAKE_CORAL))
            )
            .onFalse(Commands.runOnce({ doublePressIntakeTimer.stop() }))

        rollers.hasCoralTrigger
            .and(!prepareScoreButton)
            .whileTrue(superstructure.runGoal(Superstructure.State.PREPARE_TALL_SCORE))

        prepareScoreButton.whileTrue(
            superstructure
                .runGoal {
                    when (scoringState.target) {
                        ScoringState.ScoringTarget.L2 -> Superstructure.State.PREPARE_L2
                        ScoringState.ScoringTarget.L3 -> Superstructure.State.PREPARE_L3
                        ScoringState.ScoringTarget.L4 -> Superstructure.State.PREPARE_L4
                        ScoringState.ScoringTarget.PROCESSOR -> Superstructure.State.PREPARE_PROCESSOR
                        ScoringState.ScoringTarget.NET -> Superstructure.State.PREPARE_NET
                    }
                }
                .alongWith(
                    Commands.sequence(
                        Commands.waitUntil(driver.a().and(superstructure::atGoal)),
                        Commands.runOnce(robotPosition::resetLastScorePoseToCurrent),
                        rollers.runGoal {
                            if (scoringState.target.isAlgae) Rollers.State.SCORE_ALGAE else Rollers.State.SCORE_CORAL
                        },
                    )
                )
        )

        superstructure.defaultCommand =
            superstructure.runGoal {
                if (
                    robotPosition.isSafeToStowArm.asBoolean ||
                        superstructure.currentState == Superstructure.State.STOW ||
                        superstructure.currentState == Superstructure.State.ALGAE_STOW
                ) {
                    if (rollers.hasAlgae) {
                        Superstructure.State.ALGAE_STOW
                    } else {
                        Superstructure.State.STOW
                    }
                } else {
                    superstructure.goal
                }
            }

        operator.a().onTrue(Commands.runOnce({ scoringState.target = ScoringState.ScoringTarget.L2 }))
        operator.b().onTrue(Commands.runOnce({ scoringState.target = ScoringState.ScoringTarget.L3 }))
        operator.y().onTrue(Commands.runOnce({ scoringState.target = ScoringState.ScoringTarget.L4 }))
        operator.x().onTrue(Commands.runOnce({ scoringState.target = ScoringState.ScoringTarget.NET }))

        operator
            .povUp()
            .onTrue(Commands.runOnce({ scoringState.algaeIntakeTarget = ScoringState.AlgaeIntakeTarget.HIGH }))
        operator
            .povDown()
            .onTrue(Commands.runOnce({ scoringState.algaeIntakeTarget = ScoringState.AlgaeIntakeTarget.LOW }))

        driver
            .leftBumper()
            .and(driver.rightBumper())
            .and(!rollers.hasAlgaeTrigger)
            .whileTrue(
                superstructure
                    .runGoal {
                        when (scoringState.algaeIntakeTarget) {
                            ScoringState.AlgaeIntakeTarget.LOW -> Superstructure.State.INTAKE_ALGAE_LOW
                            ScoringState.AlgaeIntakeTarget.HIGH -> Superstructure.State.INTAKE_ALGAE_HIGH
                        }
                    }
                    .alongWith(rollers.runGoal(Rollers.State.INTAKE_ALGAE))
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

        drive.defaultCommand = drive.controllerCommand(joystickDriveController)
    }

    private var currentAuto = Commands.none()
    private val autoChoosers =
        List(5) { AutoSelector.DashboardQuestion("Option $it Chooser", "Option $it Question") }.toSet()

    private val autoChooser =
        AutoSelector(autoChoosers) {
                addQuestion("Which Auto?", { currentAuto = it }) {
                    addOption("Do Nothing", Commands::none)

                    var characterizationAuto = Commands.none()
                    addOption("Characterization", { characterizationAuto }) {
                        addQuestion("Which routine?", { characterizationAuto = it }) {
                            val driveRoutines = DrivetrainSysIdCommands(drive)
                            addOption(
                                "Drive Wheel Radius Characterization",
                                { WheelRadiusCharacterization(drive, localizer) },
                            )
                            addOption(
                                "Drive Linear SysId (Quasistatic Forward)",
                                { driveRoutines.linearQuasistaticForward },
                            )
                            addOption(
                                "Drive Linear SysId (Quasistatic Reverse)",
                                { driveRoutines.linearQuasistaticReverse },
                            )
                            addOption("Drive Linear SysId (Dynamic Forward)", { driveRoutines.linearDynamicForward })
                            addOption("Drive Linear SysId (Dynamic Reverse)", { driveRoutines.linearDynamicReverse })
                            addOption(
                                "Drive Angular SysId (Quasistatic Forward)",
                                { driveRoutines.angularQuasistaticForward },
                            )
                            addOption(
                                "Drive Angular SysId (Quasistatic Reverse)",
                                { driveRoutines.angularQuasistaticReverse },
                            )
                            addOption("Drive Angular SysId (Dynamic Forward)", { driveRoutines.angularDynamicForward })
                            addOption("Drive Angular SysId (Dynamic Reverse)", { driveRoutines.angularDynamicReverse })
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

        autoChooser.update()
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
