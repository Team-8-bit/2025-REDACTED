package org.team9432.frc2025.robot

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Second
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern

object LEDs {
    private const val LENGTH = 100
    private val leds = AddressableLED(RobotMap.LED_PORT)
    private val buffer = AddressableLEDBuffer(LENGTH)
    private val spacing = Meters.of(1.0 / 20.0)
    private val pattern = LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(Meters.of(5.0).per(Second), spacing)

    init {
        leds.setLength(LENGTH)
        leds.setData(buffer)
        leds.start()
    }

    fun update() {
        pattern.applyTo(buffer)
        leds.setData(buffer)
    }
}
