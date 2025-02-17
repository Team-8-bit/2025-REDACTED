package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class Elevator(private val io: KrakenElevatorIO) {
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

    private val currentControl = TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val motionMagicPositionControl = MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val neutralOut = NeutralOut()

    private val config =
        TalonFXConfiguration().apply {
            Slot0.kP = ElevatorConstants.gains.kP
            Slot0.kI = ElevatorConstants.gains.kI
            Slot0.kD = ElevatorConstants.gains.kD

            Slot0.kS = ElevatorConstants.gains.ffkS
            Slot0.kV = ElevatorConstants.gains.ffkV
            Slot0.kA = ElevatorConstants.gains.ffkA
            Slot0.kG = ElevatorConstants.gains.ffkG

            Slot0.GravityType = GravityTypeValue.Elevator_Static
            Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign

            MotorOutput.Inverted = InvertedValue.Clockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.PEAK_TORQUE_AMPS
            TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimit = ElevatorConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimitEnable = true

            Feedback.SensorToMechanismRatio = ElevatorConstants.MOTOR_ROTATIONS_PER_METER
        }

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
        io.setConfig(config, tries = 5, timeout = 0.5)
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
        LoggedTunableNumber.ifChanged(hashCode(), kP, kI, kD) { (kP, kI, kD) ->
            config.Slot0.kP = kP
            config.Slot0.kI = kI
            config.Slot0.kD = kD
            io.setConfig(config)
        }
        LoggedTunableNumber.ifChanged(hashCode(), ffkS, ffkG, ffkV, ffkA) { (kS, kG, kV, kA) ->
            config.Slot0.kS = kS
            config.Slot0.kG = kG
            config.Slot0.kV = kV
            config.Slot0.kA = kA
            io.setConfig(config)
        }
        LoggedTunableNumber.ifChanged(hashCode(), mmCruise, mmAccel, mmJerk) { (cruise, accel, jerk) ->
            config.MotionMagic.MotionMagicCruiseVelocity = cruise
            config.MotionMagic.MotionMagicAcceleration = accel
            config.MotionMagic.MotionMagicJerk = jerk
            io.setConfig(config)
        }

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
            Units.metersToInches(abs(inputs.leaderPositionMeters - goal.meters)),
        )
        Logger.recordOutput("Elevator/AtGoal", atGoal())
        Logger.recordOutput("Elevator/Poses/Stage2", Pose3d(0.0, 0.0, inputs.leaderPositionMeters, Rotation3d.kZero))
    }

    fun atGoal(toleranceMeters: Double = ElevatorConstants.POSITION_TOLERANCE) =
        abs(inputs.leaderPositionMeters - goal.meters) < toleranceMeters

    fun runCharacterizationAmps(amps: Double) {
        characterizing = true
        io.setControl(currentControl.withOutput(amps))
    }

    fun endCharacterization() {
        characterizing = false
    }

    fun getSpeedMps(): Double {
        return inputs.leaderVelocityMetersPerSec
    }
}
