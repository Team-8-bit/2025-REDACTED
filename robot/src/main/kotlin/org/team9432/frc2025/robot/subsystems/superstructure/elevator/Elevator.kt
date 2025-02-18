package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Constants

class Elevator(private val io: ElevatorIO) {
    private val inputs: LoggedElevatorIOInputs = LoggedElevatorIOInputs()

    private val leaderDisconnectedAlert = Alert("Leader (left) elevator motor disconnected!", Alert.AlertType.kError)
    private val followerDisconnectedAlert =
        Alert("Follower (right) elevator motor disconnected!", Alert.AlertType.kError)

    private val currentControl = TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val motionMagicPositionControl = MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val neutralOut = NeutralOut()

    private val gains: TunableElevatorGains

    enum class Goal(private val setpointSupplier: () -> Double) {
        STOW({ 0.0 }),
        TEST(LoggedTunableNumber("Elevator/Setpoints/Test", 0.0));

        val meters
            get() = setpointSupplier.invoke()
    }

    var goal = Goal.STOW
    private var characterizing = false

    var isDisabled = { DriverStation.isDisabled() }

    init {
        io.setBrake(true)

        gains =
            when (Constants.robot) {
                Constants.RobotType.COMP -> {
                    TunableElevatorGains("Elevator/Tuning", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
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
        if (!characterizing && !isDisabled()) {
            // Make sure we don't go outside the limits
            val goalPosition =
                MathUtil.clamp(goal.meters, ElevatorConstants.MIN_POSITION, ElevatorConstants.MAX_POSITION)

            // If the elevator is stowed successfully, stop both motors
            if (goal == Goal.STOW && atGoal(Units.inchesToMeters(0.25))) {
                io.setControl(neutralOut)
            } else {
                // Otherwise run to the target position
                io.setControl(motionMagicPositionControl.withPosition(goalPosition))
            }
        }

        // Diagnostic information
        Logger.recordOutput(
            "Elevator/PositionErrorInches",
            Units.metersToInches(abs(inputs.positionMeters - goal.meters)),
        )
        Logger.recordOutput("Elevator/AtGoal", atGoal())
    }

    val positionMeters
        get() = inputs.positionMeters

    val velocityMps: Double
        get() = inputs.velocityMetersPerSec

    fun atGoal(toleranceMeters: Double = ElevatorConstants.POSITION_TOLERANCE) =
        abs(inputs.positionMeters - goal.meters) < toleranceMeters

    fun runCharacterizationAmps(amps: Double) {
        characterizing = true
        io.setControl(currentControl.withOutput(amps))
    }

    fun endCharacterization() {
        characterizing = false
    }
}
