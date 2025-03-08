package org.team9432.frc2025.robot.subsystems.drive.module

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged

interface ModuleIO {
    @Logged
    open class ModuleIOInputs {
        var drivePositionRotations: Double = 0.0
        var driveVelocityRotationsPerSecond: Double = 0.0
        var driveAppliedVolts: Double = 0.0
        var driveSupplyCurrentAmps: Double = 0.0
        var driveTorqueCurrentAmps: Double = 0.0
        var driveTempFahrenheit: Double = 0.0
        var driveClosedLoopPositionReference: Double = 0.0
        var driveClosedLoopVelocityReference: Double = 0.0
        var driveClosedLoopOutput: Double = 0.0

        var steerAbsolutePosition: Rotation2d = Rotation2d()
        var steerPosition: Rotation2d = Rotation2d()
        var steerVelocityRotationsPerSec: Double = 0.0
        var steerAppliedVolts: Double = 0.0
        var steerSupplyCurrentAmps: Double = 0.0
        var steerTorqueCurrentAmps: Double = 0.0
        var steerTempFahrenheit: Double = 0.0
        var steerClosedLoopPositionReference: Double = 0.0
        var steerClosedLoopVelocityReference: Double = 0.0
        var steerClosedLoopOutput: Double = 0.0

        var odometryDrivePositionsRotations: DoubleArray = doubleArrayOf()
        var odometrySteerPositions: Array<Rotation2d> = arrayOf()

        var driveConnected: Boolean = false
        var steerConnected: Boolean = false
        var cancoderConnected: Boolean = false
    }

    /** Updates the inputs with the latest sensor information. */
    fun updateInputs(inputs: ModuleIOInputs) {}

    /** Sends the specified control request to the drive motor. */
    fun setDriveControl(control: ControlRequest) {}

    /** Sends the specified control request to the steer motor. */
    fun setSteerControl(control: ControlRequest) {}

    /** Updates the configuration of the drive motor. */
    fun updateDriveConfig(block: (TalonFXConfiguration) -> Unit) {}

    /** Updates the configuration of the steer motor. */
    fun updateSteerConfig(block: (TalonFXConfiguration) -> Unit) {}

    /** Enables or disables brake mode on the drive motor. */
    fun setDriveBrake(enable: Boolean) {}

    /** Enables or disables brake mode on the steer motor. */
    fun setSteerBrake(enable: Boolean) {}
}
