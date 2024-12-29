package org.team9432.frc2025.lib

import edu.wpi.first.wpilibj.DriverStation.Alliance

object AllianceTracker {
    /** The current alliance the robot is on, or null if no alliance has been provided yet. Must be set by user code. */
    var currentAlliance: Alliance? = null

    /** Returns either [blue] or [red] depending on the alliance color. */
    fun <T> switch(blue: T, red: T): T = if (currentAlliance == Alliance.Blue) blue else red

    /** Calls the given block if the robot is on the red alliance. */
    fun ifRed(block: () -> Unit) {
        if (currentAlliance == Alliance.Red) block.invoke()
    }

    /** Calls the given block if the robot is on the blue alliance. */
    fun ifBlue(block: () -> Unit) {
        if (currentAlliance == Alliance.Blue) block.invoke()
    }
}
