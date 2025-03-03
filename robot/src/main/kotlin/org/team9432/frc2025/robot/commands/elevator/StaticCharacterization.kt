package org.team9432.frc2025.robot.commands.elevator

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class StaticCharacterization(
    subsystem: Subsystem,
    private val inputConsumer: DoubleConsumer,
    private val velocitySupplier: DoubleSupplier,
    private val onEnd: () -> Unit,
) : Command() {
    private val timer = Timer()
    private var currentInput = 0.0

    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        timer.restart()
    }

    override fun execute() {
        currentInput = timer.get() * currentRampFactor.get()
        inputConsumer.accept(currentInput)
    }

    override fun isFinished(): Boolean {
        return velocitySupplier.asDouble >= minVelocity.get()
    }

    override fun end(interrupted: Boolean) {
        println("Static Characterization output: $currentInput amps")
        inputConsumer.accept(0.0)
        onEnd()
    }

    companion object {
        private val currentRampFactor = LoggedTunableNumber("StaticCharacterization/CurrentRampPerSec", 1.0)
        private val minVelocity = LoggedTunableNumber("StaticCharacterization/MinStaticVelocity", 0.1)
    }
}
