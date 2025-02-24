package org.team9432.frc2025.lib.util

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

/** Equivalent to [Command.addRequirements] but returns the [Command] for chaining. */
fun Command.withRequirements(vararg requirements: SubsystemBase): Command {
    addRequirements(*requirements)
    return this
}
