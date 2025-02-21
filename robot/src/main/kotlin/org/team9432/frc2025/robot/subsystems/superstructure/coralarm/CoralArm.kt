package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Constants
import kotlin.math.abs

class CoralArm(private val io: CoralArmIO) {
    private val inputs = LoggedCoralArmIOInputs()

    private val motorDisconnectedAlert = Alert("CoralArm motor disconnected!", Alert.AlertType.kError)

    private val currentControl = TorqueCurrentFOC(0.0)
    private val motionMagicPositionControl = MotionMagicTorqueCurrentFOC(0.0)
    private val neutralOut = NeutralOut()

    private val gains: TunableCoralArmGains

    // All angles are in rotations
    enum class Goal(private val angleSupplier: () -> Double) {
        STOW({ CoralArmConstants.MIN_POSITION }),
        TEST(LoggedTunableNumber("CoralArm/Setpoints/Test", 0.0));

        val rotations
            get() = angleSupplier.invoke()
    }

    var goal = Goal.STOW

    /** Characterization input in amps sent to the arm. If set to null will run position control. */
    var characterizationInput: Double? = null

    var isDisabled = { DriverStation.isDisabled() }

    init {
        io.setBrake(true)

        gains =
            when (Constants.robot) {
                Constants.RobotType.COMP ->
                    TunableCoralArmGains(
                        "CoralArm/Tuning",
                        kP = 0.0,
                        kD = 0.0,
                        kS = 0.0,
                        kG = 0.0,
                        velocity = 0.0,
                        acceleration = 0.0,
                        jerk = 0.0,
                    )

                Constants.RobotType.SIM ->
                    TunableCoralArmGains(
                        "CoralArm/Tuning",
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

    private var wasDisabled = true

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("CoralArm", inputs)

        motorDisconnectedAlert.set(!inputs.motorConnected)

        gains.ifChanged(hashCode()) { io.updateConfig { config -> gains.applyToTalonFXConfig(config) } }

        val disabled = isDisabled()

        if (disabled != wasDisabled) {
            wasDisabled = disabled
            // Coast when disabled
            io.setBrake(!disabled)
        }

        if (!disabled) {
            if (characterizationInput == null) {
                val goalPosition =
                    MathUtil.clamp(goal.rotations, CoralArmConstants.MIN_POSITION, CoralArmConstants.MAX_POSITION)

                if (goal == Goal.STOW && atGoal()) {
                    io.setControl(neutralOut)
                } else {
                    io.setControl(motionMagicPositionControl.withPosition(goalPosition))
                }
            } else {
                io.setControl(currentControl.withOutput(characterizationInput!!))
            }
        }

        Logger.recordOutput("CoralArm/Goal", goal)
    }

    val positionRotations
        get() = inputs.positionRotations

    val velocityRotationsPerSecond
        get() = inputs.velocityRotationsPerSec

    fun atGoal(toleranceRotations: Double = CoralArmConstants.POSITION_TOLERANCE) =
        abs(inputs.positionRotations - goal.rotations) < toleranceRotations
}
