package org.team9432.frc2025.lib.util

import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID

fun CommandGenericHID.rumbleCommand() =
    Commands.startEnd({ hid.setRumble(RumbleType.kBothRumble, 1.0) }, { hid.setRumble(RumbleType.kBothRumble, 0.0) })
        .asProxy()

fun CommandGenericHID.alternatingRumbleCommand(periodSeconds: Double) =
    Commands.repeatingSequence(
            Commands.runOnce({
                hid.setRumble(RumbleType.kLeftRumble, 1.0)
                hid.setRumble(RumbleType.kRightRumble, 0.0)
            }),
            Commands.waitSeconds(periodSeconds / 2),
            Commands.runOnce({
                hid.setRumble(RumbleType.kLeftRumble, 0.0)
                hid.setRumble(RumbleType.kRightRumble, 1.0)
            }),
            Commands.waitSeconds(periodSeconds / 2),
        )
        .finallyDo { _ -> hid.setRumble(RumbleType.kBothRumble, 0.0) }
        .asProxy()
