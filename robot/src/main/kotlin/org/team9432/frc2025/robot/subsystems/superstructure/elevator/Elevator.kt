package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class Elevator(private val io: ElevatorIO) {
    private val inputs: LoggedElevatorIOInputs = LoggedElevatorIOInputs()

    private val leaderDisconnectedAlert = Alert("Leader (left) elevator motor disconnected!", Alert.AlertType.kError)
    private val followerDisconnectedAlert =
        Alert("Follower (right) elevator motor disconnected!", Alert.AlertType.kError)

    private val kP = LoggedTunableNumber("Elevator/Control/kP", ElevatorConstants.gains.kP)
    private val kI = LoggedTunableNumber("Elevator/Control/kI", ElevatorConstants.gains.kI)
    private val kD = LoggedTunableNumber("Elevator/Control/kD", ElevatorConstants.gains.kD)
    private val ffkS = LoggedTunableNumber("Elevator/Control/ffkS", ElevatorConstants.gains.ffkS)
    private val ffkG = LoggedTunableNumber("Elevator/Control/ffkG", ElevatorConstants.gains.ffkG)
    private val ffkV = LoggedTunableNumber("Elevator/Control/ffkV", ElevatorConstants.gains.ffkV)
    private val ffkA = LoggedTunableNumber("Elevator/Control/ffkA", ElevatorConstants.gains.ffkA)

    private val mmCruise =
        LoggedTunableNumber("Elevator/Control/MotionMagicCruiseVelocity", ElevatorConstants.gains.mmCruise)
    private val mmAccel =
        LoggedTunableNumber("Elevator/Control/MotionMagicAcceleration", ElevatorConstants.gains.mmAccel)
    private val mmJerk = LoggedTunableNumber("Elevator/Control/MotionMagicJerk", ElevatorConstants.gains.mmJerk)

    enum class Goal(private val setpointInchesSupplier: () -> Double) {
        STOW({ 0.0 }),
        TEST(LoggedTunableNumber("Elevator/Setpoints/Test", 0.0));

        val meters
            get() = Units.inchesToMeters(setpointInchesSupplier.invoke())
    }

    var goal = Goal.STOW
    private var characterizing = false

    var isDisabled = { DriverStation.isDisabled() }

    init {
        io.setBrake(true)
    }

    fun periodic() {
        // Process log inputs
        io.updateInputs(inputs)
        Logger.processInputs("Elevator", inputs)

        // Set motor alerts
        leaderDisconnectedAlert.set(!inputs.leaderConnected)
        followerDisconnectedAlert.set(!inputs.followerConnected)

        // Update motor constants from networktables
        LoggedTunableNumber.ifChanged(hashCode(), kP, kI, kD) { (kP, kI, kD) -> io.setPID(kP, kI, kD) }
        LoggedTunableNumber.ifChanged(hashCode(), ffkS, ffkG, ffkV, ffkA) { (kS, kG, kV, kA) ->
            io.setFF(kS, kG, kV, kA)
        }
        LoggedTunableNumber.ifChanged(hashCode(), mmCruise, mmAccel, mmJerk) { (cruise, accel, jerk) ->
            io.setMotionMagic(cruise, accel, jerk)
        }

        // Run elevator
        if (!characterizing && !isDisabled()) {
            // Make sure we don't go outside the limits
            val goalPosition =
                MathUtil.clamp(goal.meters, ElevatorConstants.MIN_POSITION, ElevatorConstants.MAX_POSITION)

            // If the elevator is stowed successfully, stop both motors
            if (goal == Goal.STOW && atGoal(Units.inchesToMeters(0.25))) {
                io.stop()
            } else {
                // Otherwise run to the target position
                io.runPosition(goalPosition, feedforward = 0.0)
            }
        }

        // Diagnostic information
        Logger.recordOutput(
            "Elevator/PositionErrorInches",
            Units.metersToInches(abs(inputs.leaderPositionMeters - goal.meters)),
        )
        Logger.recordOutput("Elevator/AtGoal", atGoal())
    }

    fun atGoal(toleranceMeters: Double = ElevatorConstants.POSITION_TOLERANCE) =
        abs(inputs.leaderPositionMeters - goal.meters) < toleranceMeters

    fun runCharacterizationAmps(amps: Double) {
        characterizing = true
        io.runAmps(amps)
    }

    fun endCharacterization() {
        characterizing = false
    }

    fun getSpeedMps(): Double {
        return inputs.leaderVelocityMetersPerSec
    }
}
