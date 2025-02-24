package org.team9432.frc2025.robot.subsystems.superstructure.arm

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Constants
import org.team9432.frc2025.robot.commands.elevator.StaticCharacterization
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator.Goal
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.ElevatorConstants

class Arm(private val io: ArmIO) : SubsystemBase() {
    private val inputs = LoggedArmIOInputs()

    private val motorDisconnectedAlert = Alert("Arm motor disconnected!", Alert.AlertType.kError)

    private val voltageControl = VoltageOut(0.0)
    private val currentControl = TorqueCurrentFOC(0.0)
    private val motionMagicPositionControl = MotionMagicTorqueCurrentFOC(0.0)
    private val neutralOut = NeutralOut()

    private val gains: TunableArmGains

    // All angles are in rotations
    enum class Goal(private val angleSupplier: () -> Double) {
        STOW({ ArmConstants.MIN_POSITION }),
        PREPARE_SCORE({ 0.16 }),
        L2(LoggedTunableNumber("Arm/Setpoints/L2", 0.17)),
        L3(LoggedTunableNumber("Arm/Setpoints/L3", 0.17)),
        L4(LoggedTunableNumber("Arm/Setpoints/L4", 0.17)),
        TEST(LoggedTunableNumber("Arm/Setpoints/Test", 0.0));

        val rotations
            get() = angleSupplier.invoke()
    }

    private val homingVolts = LoggedTunableNumber("Arm/Tuning/HomingVolts", -2.0)
    private val homingTimeSecs = LoggedTunableNumber("Arm/Tuning/HomingThresholdSecs", 0.25)
    private val homingVelocityThreshold = LoggedTunableNumber("Arm/Tuning/HomingVelocityThresholdRPS", 0.1)
    private var homingDebouncer = Debouncer(homingTimeSecs.get())

    private val motorOutputDisabled = { DriverStation.isDisabled() }

    private var goal = Goal.STOW
    private var hasHomed = false

    /** Characterization input in amps sent to the arm. If set to null will run position control. */
    var characterizationInput: Double? = null

    init {
        io.setBrake(true)

        gains =
            when (Constants.robot) {
                Constants.RobotType.COMP ->
                    TunableArmGains(
                        "Arm/Gains",
                        kP = 2000.0,
                        kD = 20.0,
                        kS = 4.440481,
                        kG = 7.537810 - 4.440481,
                        velocity = 1.5,
                        acceleration = 3.0,
                        jerk = 0.0,
                    )

                Constants.RobotType.SIM ->
                    TunableArmGains(
                        "Arm/Gains",
                        kP = 3000.0,
                        kD = 300.0,
                        kS = 0.0,
                        kG = 0.0,
                        velocity = 2.0,
                        acceleration = 2.0,
                        jerk = 0.0,
                    )
            }
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Arm", inputs)

        motorDisconnectedAlert.set(!inputs.motorConnected)

        gains.ifChanged(hashCode()) { io.updateConfig { config -> gains.applyToTalonFXConfig(config) } }

        val shouldRunPosition = !motorOutputDisabled() && hasHomed && characterizationInput == null

        if (shouldRunPosition) {
            val goalPosition = MathUtil.clamp(goal.rotations, ArmConstants.MIN_POSITION, ArmConstants.MAX_POSITION)
            io.setControl(motionMagicPositionControl.withPosition(goalPosition))
        } else if (characterizationInput != null) {
            io.setControl(currentControl.withOutput(characterizationInput!!))
        }

        Logger.recordOutput("Arm/Goal", goal)
        Logger.recordOutput("Arm/CharacterizationInput", characterizationInput ?: 0.0)
    }

    val positionRotations
        get() = inputs.positionRotations

    val velocityRotationsPerSecond
        get() = inputs.velocityRotationsPerSec

    fun atGoal(toleranceRotations: Double = ArmConstants.POSITION_TOLERANCE) =
        abs(inputs.positionRotations - goal.rotations) < toleranceRotations

    /** Runs the arm down to the hardstop and resets the position. Configuration via networktables. */
    fun homeArm(): Command =
        startRun(
                /* start = */ {
                    hasHomed = false
                    homingDebouncer = Debouncer(homingTimeSecs.get())
                },
                /* run = */ { io.setControl(voltageControl.withOutput(homingVolts.get())) },
            )
            .until { homingDebouncer.calculate(abs(inputs.velocityRotationsPerSec) < homingVelocityThreshold.get()) }
            .andThen({
                io.setSensorPosition(ElevatorConstants.MIN_POSITION)
                hasHomed = true
            })
            .andThen(runOnce { this.goal = Goal.STOW })
            .onlyWhile { !motorOutputDisabled() }

    /** Runs the elevator to the given [goal] and ends when the position is reached. */
    fun runToGoal(goal: Goal) = runOnce { this.goal = goal }.andThen(Commands.idle(this)).until(::atGoal)

    fun staticCharacterization() =
        StaticCharacterization(
            subsystem = this,
            { amps -> characterizationInput = amps },
            { velocityRotationsPerSecond },
            { characterizationInput = null },
        )
}
