package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

import org.team9432.annotation.Logged

interface CoralArmIO {
    @Logged
    open class CoralArmIOInputs {
        var positionRotations: Double = 0.0
        var velocityRPM: Double = 0.0
        var appliedVoltage: Double = 0.0
        var supplyCurrentAmps: Double = 0.0
        var tempFahrenheit: Double = 0.0
    }

    fun updateInputs(inputs: CoralArmIOInputs) {}

    fun runVoltage(volts: Double) {}

    fun setBrakeMode(enabled: Boolean) {}
}
