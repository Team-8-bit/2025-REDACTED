package org.team9432.frc2025.robot.subsystems.drive.module

import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged

interface ModuleIO {
    @Logged
    open class ModuleIOInputs {
        var drivePositionRads: Double = 0.0
        var driveVelocityRadPerSecond: Double = 0.0
        var driveAppliedVolts: Double = 0.0
        var driveSupplyCurrentAmps: Double = 0.0
        var driveTorqueCurrentAmps: Double = 0.0

        var steerAbsolutePosition: Rotation2d = Rotation2d()
        var steerPosition: Rotation2d = Rotation2d()
        var steerVelocityRadPerSec: Double = 0.0
        var steerAppliedVolts: Double = 0.0
        var steerSupplyCurrentAmps: Double = 0.0
        var steerTorqueCurrentAmps: Double = 0.0

        var odometryDrivePositionsRads: DoubleArray = doubleArrayOf()
        var odometrySteerPositions: Array<Rotation2d> = arrayOf()

        var driveConnected: Boolean = false
        var steerConnected: Boolean = false
        var cancoderConnected: Boolean = false
    }

    /** Updates the inputs with the latest sensor information. */
    fun updateInputs(inputs: ModuleIOInputs) {}

    /** Runs the drive motor at the specified voltage. */
    fun runDriveVoltage(volts: Double) {}

    /** Runs the steer motor at the specified voltage. */
    fun runSteerVoltage(volts: Double) {}

    /** Runs the drive motor at the specified current. */
    fun runDriveAmps(amps: Double) {}

    /** Runs the steer motor at the specified current. */
    fun runSteerAmps(amps: Double) {}

    /** Runs the drive motor at the specified velocity with the given feedforward. */
    fun runDriveVelocity(velocityRadPerSec: Double, feedforward: Double) {}

    /** Runs the steer motor to the specified position. */
    fun runSteerPosition(angle: Rotation2d) {}

    /** Sets the pid constants of the drive motor. */
    fun setDrivePID(p: Double, i: Double, d: Double) {}

    /** Sets the pid constants of the steer motor. */
    fun setSteerPID(p: Double, i: Double, d: Double) {}

    /** Enables or disables brake mode on the drive motor. */
    fun setDriveBrake(enable: Boolean) {}

    /** Enables or disables brake mode on the steer motor. */
    fun setSteerBrake(enable: Boolean) {}
}
