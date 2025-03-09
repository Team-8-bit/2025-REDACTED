package org.team9432.frc2025.robot

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Second
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color

object LEDs {
    private const val LENGTH = 46
    private val leds = AddressableLED(RobotMap.LED_PORT)
    private val buffer = AddressableLEDBuffer(LENGTH)
    private val spacing = Meters.of(1.0 / 20.0)
    private val pattern = LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(Meters.of(1.0).per(Second), spacing)
    private val teamPattern =
        LEDPattern.gradient(
                LEDPattern.GradientType.kContinuous,
                Color.kPurple,
                Color.kBlack,
                Color.kBlack,
                Color.kBlack,
                Color.kBlack,
                Color.kBlack,
                Color.kBlack,
                Color.kBlack,
            )
            .scrollAtAbsoluteSpeed(Meters.of(3.0).per(Second), spacing)
    private val BISQUE = LEDPattern.solid(Color.kBisque)

    init {
        leds.setLength(LENGTH)
        leds.setData(buffer)
        leds.start()
    }

    fun update() {
        //        BISQUE.applyTo(buffer)
        teamPattern.applyTo(buffer)

        // Swap red and green for the led strips we're using
        for (i in 0..<LENGTH) {
            val r = buffer.getRed(i)
            val g = buffer.getGreen(i)
            val b = buffer.getBlue(i)

            buffer.setRGB(i, g, r, b)
        }

        leds.setData(buffer)
    }
}
