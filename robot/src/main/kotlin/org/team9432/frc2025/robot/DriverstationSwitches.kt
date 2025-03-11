package org.team9432.frc2025.robot

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID

class DriverstationSwitches(port: Int) : CommandGenericHID(port) {
    val one
        get() = super.button(6)

    //    val two
    //        get() = super.button(9) // :(

    val three
        get() = super.button(5)

    val four
        get() = super.button(7)

    //    val five get() = super.button(-1) // Broken
    val six
        get() = super.button(3)

    val seven
        get() = super.button(1)

    val eight
        get() = super.button(2)
}
