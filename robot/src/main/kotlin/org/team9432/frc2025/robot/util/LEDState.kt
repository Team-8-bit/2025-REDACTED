package org.team9432.frc2025.robot.util

import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import org.team9432.frc2025.lib.AllianceTracker

object LEDState {
    private val spacing = Meters.of(1.0 / 20.0)
    private val idlePattern =
        LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBisque, Color.kPurple)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(1.0), spacing)

    private const val INIT_LOOP_COUNT = 200
    private var initLoops = 0

    private val codeLoadingPattern = LEDPattern.solid(Color.kBisque)
    private val climbModePattern = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.25), Seconds.of(0.25))
    private val autoAlignPattern = LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.5), Seconds.of(0.5))
    private val visionDisconnectedPattern = LEDPattern.solid(Color.kRed)

    private val redPattern =
        LEDPattern.steps(mutableMapOf(0.0 to Color.kRed, 5.0 / 46.0 to Color.kBlack))
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(6.0), spacing)
    private val bluePattern =
        LEDPattern.steps(mutableMapOf(0.0 to Color.kBlue, 5.0 / 46.0 to Color.kBlack))
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(6.0), spacing)

    private val seesDisabledTagPattern = LEDPattern.solid(Color.kForestGreen).breathe(Seconds.of(3.0))

    private val elevatorHeightPattern =
        LEDPattern.solid(Color.kBisque).mask(LEDPattern.progressMaskLayer { elevatorHeight() })

    var codeLoading = false
    var climbMode = false
    var isAutoAligning = { false }

    var visionDisconnected = { false }
    var seesDisabledTag = { false }

    var displayElevatorHeight = { false }
    var elevatorHeight = { 0.0 }

    var shouldRunDisplay = { false }

    fun updateBuffer(buffer: AddressableLEDBuffer) {
        if (initLoops < INIT_LOOP_COUNT) {
            initLoops++
            climbModePattern.applyTo(buffer)
            autoAlignPattern.applyTo(buffer)
            redPattern.applyTo(buffer)
            bluePattern.applyTo(buffer)
            codeLoadingPattern.applyTo(buffer)

            if (codeLoading) {
                codeLoadingPattern.applyTo(buffer)
            }

            return
        }

        if (codeLoading) {
            codeLoadingPattern.applyTo(buffer)
        } else if (displayElevatorHeight()) {
            elevatorHeightPattern.applyTo(LEDStrip.leftSection)
            elevatorHeightPattern.reversed().applyTo(LEDStrip.rightSection)
        } else if (seesDisabledTag()) {
            seesDisabledTagPattern.applyTo(buffer)
        } else if (climbMode) {
            climbModePattern.applyTo(buffer)
        } else if (visionDisconnected()) {
            visionDisconnectedPattern.applyTo(buffer)
        } else if (isAutoAligning()) {
            autoAlignPattern.applyTo(buffer)
        } else {
            if (shouldRunDisplay()) {
                idlePattern.applyTo(buffer)
            } else {
                when (AllianceTracker.currentAlliance) {
                    null -> idlePattern.applyTo(buffer)
                    Alliance.Red -> redPattern.applyTo(buffer)
                    Alliance.Blue -> bluePattern.applyTo(buffer)
                }
            }
        }
    }
}
