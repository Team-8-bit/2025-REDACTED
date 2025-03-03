package org.team9432.frc2025.lib.util

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger

/** Equivalent to [Command.addRequirements] but returns the [Command] for chaining. */
fun Command.withRequirements(vararg requirements: SubsystemBase): Command {
    addRequirements(*requirements)
    return this
}

/** Equivalent to [Trigger.negate]. */
operator fun Trigger.not(): Trigger = this.negate()
