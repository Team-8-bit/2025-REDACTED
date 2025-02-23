package org.team9432.frc2025.robot.subsystems.superstructure.arm

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Constants

class Arm(private val io: ArmIO) {
    private val inputs = LoggedArmIOInputs()

    private val motorDisconnectedAlert = Alert("Arm motor disconnected!", Alert.AlertType.kError)

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

    var goal = Goal.STOW

    /** Characterization input in amps sent to the arm. If set to null will run position control. */
    var characterizationInput: Double? = null

    var isDisabled = { DriverStation.isDisabled() }

    init {
        io.setBrake(true)

        gains =
            when (Constants.robot) {
                Constants.RobotType.COMP ->
                    TunableArmGains(
                        "Arm/Tuning",
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
                        "Arm/Tuning",
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

    private var wasDisabled = false

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Arm", inputs)

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
                val goalPosition = MathUtil.clamp(goal.rotations, ArmConstants.MIN_POSITION, ArmConstants.MAX_POSITION)

                if (goal == Goal.STOW && atGoal()) {
                    io.setControl(neutralOut)
                } else {
                    io.setControl(motionMagicPositionControl.withPosition(goalPosition))
                }
            } else {
                io.setControl(currentControl.withOutput(characterizationInput!!))
            }
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
}
