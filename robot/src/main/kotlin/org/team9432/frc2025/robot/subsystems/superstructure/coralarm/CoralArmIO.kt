package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import org.team9432.annotation.Logged

interface CoralArmIO {
    @Logged
    open class CoralArmIOInputs {
        var positionRotations: Double = 0.0
        var velocityRotationsPerSec: Double = 0.0
        var closedLoopPositionReference: Double = 0.0
        var closedLoopVelocityReference: Double = 0.0
        var closedLoopOutput: Double = 0.0

        var motorConnected: Boolean = true
        var appliedVolts: Double = 0.0
        var supplyCurrentAmps: Double = 0.0
        var torqueCurrentAmps: Double = 0.0
        var tempFahrenheit: Double = 0.0
    }

    /** Updates the inputs with the latest sensor information. */
    fun updateInputs(inputs: CoralArmIOInputs) {}

    /** Sends the specified control request to the motor. */
    fun setControl(control: ControlRequest) {}

    /** Updates the configuration of the motor. */
    fun updateConfig(block: (TalonFXConfiguration) -> Unit) {}

    fun setBrake(enable: Boolean) {}
}
