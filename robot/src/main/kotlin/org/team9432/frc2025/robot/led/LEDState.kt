package org.team9432.frc2025.robot.led

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color

object LEDState {
    private val spacing = Meters.of(1.0 / 20.0)
    private val idlePattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBisque, Color.kPurple)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(1.0), spacing)

    fun updateBuffer(buffer: AddressableLEDBuffer) {
        idlePattern.applyTo(buffer)
    }
}
