package org.team9432.frc2025.robot.led

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer

class LEDStrip(pwm: Int, private val length: Int) {
    private val leds = AddressableLED(pwm)
    val buffer = AddressableLEDBuffer(length)

    init {
        leds.setLength(length)
        leds.setData(buffer)
        leds.start()
    }

    fun displayBuffer() {
        // Swap red and green for the led strips we're using
        for (i in 0..<length) {
            val r = buffer.getRed(i)
            val g = buffer.getGreen(i)
            val b = buffer.getBlue(i)

            buffer.setRGB(i, g, r, b)
        }

        leds.setData(buffer)
    }
}
