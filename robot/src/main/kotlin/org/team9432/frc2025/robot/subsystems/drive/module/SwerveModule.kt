package org.team9432.frc2025.robot.subsystems.drive.module

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants


class SwerveModule(private val io: ModuleIO, private val name: String) {
    private val inputs: LoggedModuleIOInputs = LoggedModuleIOInputs()

    private val driveDisconnectedAlert = Alert("$name drive disconnected!", AlertType.kError)
    private val steerDisconnectedAlert = Alert("$name steer disconnected!", AlertType.kError)
    private val cancoderDisconnectedAlert = Alert("$name cancoder disconnected!", AlertType.kError)

    private var driveFeedforward = SimpleMotorFeedforward(0.0, 0.0)
    var currentSetpoint: SwerveModuleState = SwerveModuleState()
        private set

    init {
        io.setDriveBrake(true)
        io.setSteerBrake(true)
    }

    fun updateInputs() {
        io.updateInputs(inputs)
        Logger.processInputs("Drive/${name}", inputs)

        DrivetrainConstants.checkDriveFFChange(hashCode()) { kS, kV -> driveFeedforward = SimpleMotorFeedforward(kS, kV) }
        DrivetrainConstants.checkDrivePidChange(hashCode()) { kP, kD -> io.setDrivePID(kP, 0.0, kD) }
        DrivetrainConstants.checkSteerPidChange(hashCode()) { kP, kD -> io.setSteerPID(kP, 0.0, kD) }

        driveDisconnectedAlert.set(!inputs.driveConnected)
        steerDisconnectedAlert.set(!inputs.steerConnected)
        cancoderDisconnectedAlert.set(!inputs.cancoderConnected)
    }

    /** Get an array of the module positions recorded in the last call to [updateInputs]. */
    fun getOdometryModulePositions(): Array<SwerveModulePosition> {
        return Array(odometrySampleSize) { index ->
            val positionMeters = driveWheelRadsToMeters(inputs.odometryDrivePositionsRads[index])
            val angle = inputs.odometrySteerPositions[index]
            SwerveModulePosition(positionMeters, angle)
        }
    }

    /** Runs characterization by locking the module at [turnSetpoint] and running the drive motor at [voltage]. */
    fun runDriveCharacterizationVoltage(voltage: Double, turnSetpoint: Rotation2d) {
        io.runSteerPosition(turnSetpoint)
        io.runDriveVoltage(voltage)
    }

    /** Runs the module to the specified setpoint state. */
    fun runSetpoint(setpoint: SwerveModuleState, torqueFF: SwerveModuleState) {
        currentSetpoint = setpoint
        val wheelTorqueNm = torqueFF.speedMetersPerSecond

        val velocityRadPerSec = setpoint.speedMetersPerSecond / Units.inchesToMeters(DrivetrainConstants.WHEEL_RADIUS_INCHES)
        val feedforward = driveFeedforward.calculate(velocityRadPerSec) + ((wheelTorqueNm / DrivetrainConstants.DRIVE_RATIO) * DrivetrainConstants.ffkT)
        io.runDriveVelocity(velocityRadPerSec, feedforward)
        io.runSteerPosition(setpoint.angle)
    }

    /** The current angle of the module. */
    val angle get() = inputs.steerAbsolutePosition

    /** Current module state as reported by the robot's sensors. */
    val measuredState get() = SwerveModuleState(driveWheelRadsToMeters(inputs.driveVelocityRadPerSecond), angle)

    /** Current module position as reported by the robot's sensors. */
    val measuredPosition get() = SwerveModulePosition(driveWheelRadsToMeters(inputs.drivePositionRads), angle)

    /** Current wheel position in radians, used for characterization routines. */
    val characterizationWheelPositionRads get() = inputs.drivePositionRads

    /** The number of cached odometry readings. */
    val odometrySampleSize get() = minOf(inputs.odometryDrivePositionsRads.size, inputs.odometrySteerPositions.size)

    /** Converts rotations of the drive wheel into meters travelled */
    private fun driveWheelRadsToMeters(driveWheelRads: Double): Double {
        return driveWheelRads * Units.inchesToMeters(DrivetrainConstants.WHEEL_RADIUS_INCHES)
    }
}