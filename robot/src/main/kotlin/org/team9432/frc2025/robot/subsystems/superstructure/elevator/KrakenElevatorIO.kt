package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import org.team9432.annotation.Logged

interface KrakenElevatorIO {
    @Logged
    open class ElevatorIOInputs {
        var leaderConnected: Boolean = true
        var leaderPositionMeters: Double = 0.0
        var leaderVelocityMetersPerSec: Double = 0.0
        var leaderAppliedVolts: Double = 0.0
        var leaderSupplyCurrentAmps: Double = 0.0
        var leaderTorqueCurrentAmps: Double = 0.0
        var leaderTempFahrenheit: Double = 0.0

        var followerConnected: Boolean = true
        var followerPositionMeters: Double = 0.0
        var followerVelocityMetersPerSec: Double = 0.0
        var followerAppliedVolts: Double = 0.0
        var followerSupplyCurrentAmps: Double = 0.0
        var followerTorqueCurrentAmps: Double = 0.0
        var followerTempFahrenheit: Double = 0.0

        var closedLoopPositionReference: Double = 0.0
        var closedLoopVelocityReference: Double = 0.0
        var closedLoopOutput: Double = 0.0
    }

    /** Updates the inputs with the latest sensor information. */
    fun updateInputs(inputs: ElevatorIOInputs) {}

    /** Sends the specified control request to the leader motor. */
    fun setControl(control: ControlRequest) {}

    fun setConfig(config: TalonFXConfiguration, tries: Int = 1, timeout: Double = 0.1) {}

    /** Enables or disables brake mode on the motors. */
    fun setBrake(enable: Boolean) {}
}
