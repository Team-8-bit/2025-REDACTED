package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Constants
import org.team9432.frc2025.robot.commands.elevator.StaticCharacterization
import org.team9432.frc2025.robot.led.LEDState
import org.team9432.frc2025.robot.subsystems.superstructure.SuperstructureConstants

class Elevator(private val io: ElevatorIO) : SubsystemBase() {
    private val inputs: LoggedElevatorIOInputs = LoggedElevatorIOInputs()

    private val leaderDisconnectedAlert = Alert("Leader (left) elevator motor disconnected!", Alert.AlertType.kError)
    private val followerDisconnectedAlert =
        Alert("Follower (right) elevator motor disconnected!", Alert.AlertType.kError)

    private val currentControl = TorqueCurrentFOC(0.0)
    private val voltageControl = VoltageOut(0.0)
    private val motionMagicPositionControl = MotionMagicTorqueCurrentFOC(0.0)
    private val neutralOut = NeutralOut()

    private val gains: TunableElevatorGains

    enum class Goal(private val setpointSupplier: () -> Double) {
        STOW({ 0.0 }),
        MIN_ARM_OUT(
            LoggedTunableNumber(
                "Elevator/Setpoints/MinArmOut",
                SuperstructureConstants.MIN_ARM_EXTENSION_ELEVATOR_HEIGHT,
            )
        ),
        L1(LoggedTunableNumber("Elevator/Setpoints/L1", 0.2)),
        L2(LoggedTunableNumber("Elevator/Setpoints/L2", 0.54)),
        L3(LoggedTunableNumber("Elevator/Setpoints/L3", 0.97)),
        L4(LoggedTunableNumber("Elevator/Setpoints/L4", 1.3)),
        INTAKE_ALGAE_REEF_LOW(LoggedTunableNumber("Elevator/Setpoints/IntakeAlgaeReefLow", 0.5)),
        INTAKE_ALGAE_REEF_HIGH(LoggedTunableNumber("Elevator/Setpoints/IntakeAlgaeReefHigh", 0.9)),
        HOLD_ALGAE_LOW(LoggedTunableNumber("Elevator/Setpoints/HoldAlgaeLow", 0.1)),
        PREPARE_PROCESSOR(LoggedTunableNumber("Elevator/Setpoints/PrepareProcessor", 0.1)),
        PREPARE_NET(LoggedTunableNumber("Elevator/Setpoints/PrepareNet", ElevatorConstants.MAX_POSITION)),
        SCORE_NET(LoggedTunableNumber("Elevator/Setpoints/ScoreNet", ElevatorConstants.MAX_POSITION));

        val meters
            get() = setpointSupplier.invoke()
    }

    private val homingVolts = LoggedTunableNumber("Elevator/Tuning/HomingVolts", -1.0)
    private val homingTimeSecs = LoggedTunableNumber("Elevator/Tuning/HomingThresholdSecs", 0.25)
    private val homingVelocityThreshold =
        LoggedTunableNumber("Elevator/Tuning/HomingVelocityThresholdMPS", Units.inchesToMeters(3.0))
    private var homingDebouncer = Debouncer(homingTimeSecs.get())

    private val motorOutputDisabled = { DriverStation.isDisabled() }

    var coastOverride = { false }
    private var wasCoast = false

    private var goal = Goal.STOW
    var hasHomed = false
        private set

    /** Characterization input in amps sent to the elevator. If set to null will run position control. */
    private var characterizationInput: Double? = null

    init {
        io.setBrake(true)

        gains =
            when (Constants.robot) {
                Constants.RobotType.COMP -> {
                    TunableElevatorGains(
                        "Elevator/Gains",
                        kP = 6000.0,
                        kD = 200.0,
                        kSStage1 = 12.0,
                        kGStage1 = 1.0,
                        kSStage2 = 11.2 - 4.5,
                        kGStage2 = 4.5, // 11.2 to go up, 4.5 to go down
                        velocity = 3.0,
                        acceleration = 8.0,
                        jerk = 0.0,
                    )
                }

                Constants.RobotType.SIM -> {
                    TunableElevatorGains(
                        "Elevator/Gains",
                        kP = 500.0,
                        kD = 120.0,
                        kSStage1 = 0.0,
                        kGStage1 = 9.9,
                        kSStage2 = 0.0,
                        kGStage2 = 9.9,
                        velocity = 2.0,
                        acceleration = 8.0,
                        jerk = 0.0,
                    )
                }
            }

        LEDState.elevatorHeight = { inputs.positionMeters / ElevatorConstants.MAX_POSITION }
    }

    override fun periodic() {
        // Process log inputs
        io.updateInputs(inputs)
        Logger.processInputs("Elevator", inputs)

        // Set motor alerts
        leaderDisconnectedAlert.set(!inputs.leaderConnected)
        followerDisconnectedAlert.set(!inputs.followerConnected)

        // Update motor constants from networktables
        gains.ifChanged(hashCode()) { io.updateConfig { config -> gains.applyToTalonFXConfig(config) } }

        // Make sure we should run position control
        val shouldCoast = coastOverride()
        val shouldRunPosition = !motorOutputDisabled() && hasHomed && characterizationInput == null && !shouldCoast

        if (shouldRunPosition) {
            // Make sure we don't go outside the limits
            val goalPosition =
                MathUtil.clamp(goal.meters, ElevatorConstants.MIN_POSITION, ElevatorConstants.MAX_POSITION)

            // If the elevator is stowed successfully, stop both motors
            if (goal == Goal.STOW && atGoal()) {
                io.setControl(neutralOut)
            } else {
                // Otherwise run to the target position
                io.setControl(motionMagicPositionControl.withPosition(goalPosition))
            }
        } else if (characterizationInput != null) {
            io.setControl(currentControl.withOutput(characterizationInput!!))
        }

        if (shouldCoast != wasCoast) {
            wasCoast = shouldCoast
            io.setBrake(!shouldCoast)
        }

        // Diagnostic information
        Logger.recordOutput("Elevator/Goal", goal)
        Logger.recordOutput("Elevator/AtGoal", atGoal())
        Logger.recordOutput("Elevator/Homed", hasHomed)
        Logger.recordOutput("Elevator/CharacterizationInput", characterizationInput ?: 0.0)
        Logger.recordOutput("Elevator/RunningPositionControl", shouldRunPosition)
    }

    val positionMeters
        get() = inputs.positionMeters

    val velocityMps: Double
        get() = inputs.velocityMetersPerSec

    fun atGoal(toleranceMeters: Double = ElevatorConstants.POSITION_TOLERANCE) =
        abs(inputs.positionMeters - goal.meters) < toleranceMeters

    /** Runs the elevator down to the bottom and resets the position. Configuration via networktables. */
    fun homeElevator(): Command =
        startRun(
                /* start = */ {
                    hasHomed = false
                    homingDebouncer = Debouncer(homingTimeSecs.get())
                },
                /* run = */ { io.setControl(voltageControl.withOutput(homingVolts.get())) },
            )
            .until { homingDebouncer.calculate(abs(inputs.velocityMetersPerSec) < homingVelocityThreshold.get()) }
            .andThen({
                io.setSensorPosition(0.0)
                hasHomed = true
            })
            .andThen(runOnce { this.goal = Goal.STOW })
            .onlyWhile { !motorOutputDisabled() }

    fun fakeAutoHome(): Command = runOnce { hasHomed = true }

    /** Runs the elevator to the given [goal] and ends when the position is reached. */
    fun runToGoal(goal: Goal) = run { this.goal = goal }.until(::atGoal)

    fun staticCharacterization() =
        StaticCharacterization(
            subsystem = this,
            { amps -> characterizationInput = amps },
            { velocityMps },
            { characterizationInput = null },
        )
}
