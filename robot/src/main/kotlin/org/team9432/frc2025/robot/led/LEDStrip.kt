package org.team9432.frc2025.robot.led

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import org.team9432.frc2025.robot.RobotMap

object LEDStrip {
    private val length = 46
    private val leds = AddressableLED(RobotMap.LED_PORT)
    val buffer = AddressableLEDBuffer(length)

    val leftSection = buffer.createView(0, length / 2 - 1)
    val rightSection = buffer.createView(length / 2, length - 1)

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
