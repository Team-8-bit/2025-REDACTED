package org.team9432.frc2025.robot.subsystems.superstructure.elevator

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

class Elevator(private val io: ElevatorIO) {
    private val inputs: LoggedElevatorIOInputs = LoggedElevatorIOInputs()

    private val leaderDisconnectedAlert = Alert("Leader (left) elevator motor disconnected!", Alert.AlertType.kError)
    private val followerDisconnectedAlert =
        Alert("Follower (right) elevator motor disconnected!", Alert.AlertType.kError)

    private val currentControl = TorqueCurrentFOC(0.0)
    private val motionMagicPositionControl = MotionMagicTorqueCurrentFOC(0.0)
    private val neutralOut = NeutralOut()

    private val gains: TunableElevatorGains

    enum class Goal(private val setpointSupplier: () -> Double) {
        STOW({ 0.0 }),
        TEST(LoggedTunableNumber("Elevator/Setpoints/Test", 0.0)),
        AMP_INPUT(LoggedTunableNumber("Elevator/Control/AmpInput", 0.0));

        val meters
            get() = setpointSupplier.invoke()
    }

    var goal = Goal.STOW

    /** Characterization input in amps sent to the elevator. If set to null will run position control. */
    private var characterizationInput: Double? = null

    var isDisabled = { DriverStation.isDisabled() }

    init {
        io.setBrake(true)

        gains =
            when (Constants.robot) {
                Constants.RobotType.COMP -> {
                    TunableElevatorGains(
                        "Elevator/Tuning",
                        kP = 1200.0,
                        kD = 50.0,
                        kSStage1 = 10.8,
                        kGStage1 = 0.0,
                        kSStage2 = 11.2 - 4.5,
                        kGStage2 = 4.5, // 11.2 to go up, 4.5 to go down
                        velocity = 2.0,
                        acceleration = 6.0,
                        jerk = 0.0,
                    )
                }

                Constants.RobotType.SIM -> {
                    TunableElevatorGains(
                        "Elevator/Tuning",
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
    }

    private var wasDisabled = true

    fun periodic() {
        // Process log inputs
        io.updateInputs(inputs)
        Logger.processInputs("Elevator", inputs)

        // Set motor alerts
        leaderDisconnectedAlert.set(!inputs.leaderConnected)
        followerDisconnectedAlert.set(!inputs.followerConnected)

        // Update motor constants from networktables
        gains.ifChanged(hashCode()) { io.updateConfig { config -> gains.applyToTalonFXConfig(config) } }

        // Run elevator
        val disabled = isDisabled()

        if (!disabled) {
            if (characterizationInput == null) {
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
            } else {
                io.setControl(currentControl.withOutput(characterizationInput!!))
            }
        }

        // Diagnostic information
        Logger.recordOutput("Elevator/AtGoal", atGoal())
    }

    val positionMeters
        get() = inputs.positionMeters

    val velocityMps: Double
        get() = inputs.velocityMetersPerSec

    fun atGoal(toleranceMeters: Double = ElevatorConstants.POSITION_TOLERANCE) =
        abs(inputs.positionMeters - goal.meters) < toleranceMeters
}
