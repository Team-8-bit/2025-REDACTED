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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team9432.frc2025.lib.AllianceTracker
import org.team9432.frc2025.lib.dashboard.AutoSelector
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.commands.drive.DrivetrainSysIdCommands
import org.team9432.frc2025.robot.commands.drive.WheelRadiusCharacterization
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
import org.team9432.frc2025.robot.subsystems.drive.module.ModuleIOKraken
import org.team9432.frc2025.robot.subsystems.drive.module.ModuleIOSim

class Robot : LoggedRobot() {
    private val controller = CommandXboxController(0)

    private val drive: Drive
    private val setSimulationPose: ((Pose2d) -> Unit)?
    private val driveSim: SwerveDriveSimulation?
    private val robotState = RobotState()

    init {
        LoggedTunableNumber.setTuningModeEnabled(true)

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
                            ModuleIOKraken(ModuleConfig.FRONT_LEFT, odometryThread),
                            ModuleIOKraken(ModuleConfig.FRONT_RIGHT, odometryThread),
                            ModuleIOKraken(ModuleConfig.BACK_LEFT, odometryThread),
                            ModuleIOKraken(ModuleConfig.BACK_RIGHT, odometryThread),
                            odometryThread,
                            robotState,
                        )

                    setSimulationPose = null
                    driveSim = null
                }

                Constants.RobotType.SIM -> {
                    val swerveSim =
                        SwerveDriveSimulation(
                            DriveTrainSimulationConfig.Default()
                                .withSwerveModule {
                                    SwerveModuleSimulation(
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
                                        /* tireCoefficientOfFriction = */ COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
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
                            robotState,
                        )

                    driveSim = swerveSim
                    SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim)

                    setSimulationPose = {
                        swerveSim.setSimulationWorldPose(it)
                        gyroIO.setAngle(it.rotation)
                    }
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
                    robotState,
                )

            setSimulationPose = null
            driveSim = null
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
        val joystickDriveController =
            JoystickDriveController(
                controllerX = { -controller.leftY },
                controllerY = { -controller.leftX },
                controllerR = { controller.leftTriggerAxis - controller.rightTriggerAxis },
                robotState,
            )

        val alignStraightController =
            JoystickAimAtAngleController(joystickDriveController, { Rotation2d.kZero }, robotState)

        drive.defaultCommand = drive.controllerCommand(joystickDriveController)

        controller.a().whileTrue(drive.controllerCommand(alignStraightController))
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
                                { WheelRadiusCharacterization(drive, robotState) },
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
        Logger.recordMetadata("TuningMode", LoggedTunableNumber.isTuningModeEnabled().toString())
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

        // Log actual sim robot position
        if (Constants.robot.isSim) {
            Logger.recordOutput("SimulationArena/ActualRobotPosition", driveSim!!.simulatedDriveTrainPose)
        }

        // Log robot state
        robotState.log()

        autoChooser.update()
    }

    override fun simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic()
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
