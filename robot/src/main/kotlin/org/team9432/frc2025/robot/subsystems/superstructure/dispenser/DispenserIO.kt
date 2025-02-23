package org.team9432.frc2025.robot.subsystems.superstructure.dispenser

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import org.team9432.annotation.Logged

interface DispenserIO {
    @Logged
    open class DispenserIOInputs {
        var motorConnected: Boolean = true
        var appliedVolts: Double = 0.0
        var supplyCurrentAmps: Double = 0.0
        var torqueCurrentAmps: Double = 0.0
        var tempFahrenheit: Double = 0.0
        var positionRotations: Double = 0.0
        var velocityRotationsPerSec: Double = 0.0
    }

    /** Updates the inputs with the latest sensor information. */
    fun updateInputs(inputs: DispenserIOInputs) {}

    /** Sends the specified control request to the motor. */
    fun setControl(control: ControlRequest) {}

    /** Updates the configuration of the motor. */
    fun updateConfig(block: (TalonFXConfiguration) -> Unit) {}

    fun setBrake(enable: Boolean) {}
}
