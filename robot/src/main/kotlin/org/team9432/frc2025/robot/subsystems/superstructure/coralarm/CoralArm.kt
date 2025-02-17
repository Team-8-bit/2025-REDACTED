package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.startEnd
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class CoralArm(private val io: CoralArmIO) {
    private val inputs = LoggedCoralArmIOInputs()

    private val motorDisconnectedAlert = Alert("CoralArm motor disconnected!", Alert.AlertType.kError)

    private val kP = LoggedTunableNumber("CoralArm/Control/kP", 0.0)
    private val kI = LoggedTunableNumber("CoralArm/Control/kI", 0.0)
    private val kD = LoggedTunableNumber("CoralArm/Control/kD", 0.0)
    private val kS = LoggedTunableNumber("CoralArm/Control/kS", 0.0)
    private val kG = LoggedTunableNumber("CoralArm/Control/kG", 0.0)
    private val kV = LoggedTunableNumber("CoralArm/Control/kV", 0.0)
    private val kA = LoggedTunableNumber("CoralArm/Control/kA", 0.0)

    private val maxVelocity = LoggedTunableNumber("CoralArm/Control/MaxVelocityRotationsPerSec", 0.0)
    private val maxAcceleration = LoggedTunableNumber("CoralArm/Control/MaxAccelerationRotationsPerSecPerSec", 0.0)

    // All angles are in rotations
    enum class Goal(private val angleSupplier: () -> Double) {
        STOW({ 0.0 }),
        TEST(LoggedTunableNumber("CoralArm/Setpoints/Test", 0.0));

        val rotations
            get() = angleSupplier.invoke()
    }

    var goal = Goal.STOW
    private var characterizing = false

    private val feedback = PIDController(kP.get(), 0.0, kD.get())
    private var feedforward = ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get())

    private var profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()))
    private var setpointState = TrapezoidProfile.State()

    init {
        io.setBrakeMode(true)
    }

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("CoralArm", inputs)

        LoggedTunableNumber.ifChanged(hashCode(), kP, kI, kD) { (kP, kI, kD) -> feedback.setPID(kP, kI, kD) }
        LoggedTunableNumber.ifChanged(hashCode(), kS, kG, kV, kA) { (kS, kG, kV, kA) ->
            feedforward = ArmFeedforward(kS, kG, kV, kA)
        }
        LoggedTunableNumber.ifChanged(hashCode(), maxVelocity, maxAcceleration) { (maxVel, maxAcc) ->
            profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVel, maxAcc))
        }

        val disabled = DriverStation.isDisabled()

        if (disabled) {
            io.runVoltage(0.0)
            feedback.reset()
            setpointState = TrapezoidProfile.State(0.0, 0.0)
        }

        io.setBrakeMode(!disabled) // Coast when disabled

        if (!characterizing && !disabled) {
            setpointState =
                profile.calculate(
                    LoggedRobot.defaultPeriodSecs,
                    setpointState,
                    TrapezoidProfile.State(
                        MathUtil.clamp(goal.rotations, CoralArmConstants.MIN_POSITION, CoralArmConstants.MAX_POSITION),
                        0.0,
                    ),
                )

            io.runVoltage(
                feedforward.calculate(Units.rotationsToRadians(positionRotations), setpointState.velocity) +
                    feedback.calculate(positionRotations, setpointState.position)
            )
        }

        Logger.recordOutput("CoralArm/Goal", goal)
        Logger.recordOutput("CoralArm/PositionRotations", positionRotations)
        Logger.recordOutput("CoralArm/GoalRotations", goal.rotations)
        Logger.recordOutput("CoralArm/SetpointPositionRotations", setpointState.position)
        Logger.recordOutput("CoralArm/SetpointVelocityRotationsPerSec", setpointState.velocity)
    }

    fun runGoal(newGoal: Goal): Command =
        startEnd({ this.goal = newGoal }, { goal = Goal.STOW }).withName("Coral Arm $goal")

    val positionRotations
        get() = inputs.positionRotations

    fun runCharacterization(volts: Double) {
        characterizing = true
        io.runVoltage(volts)
    }

    fun endCharacterization() {
        characterizing = false
    }
}
