package org.team9432.frc2025.lib.constants

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.util.Units

/**
 * - Robot rotation is always 0 degrees when the front of the robot is facing the red alliance wall
 * - (0,0) coordinate on the far right of the blue driver station wall (as though you were standing behind it)
 * - +x is towards the red alliance wall
 * - +y is towards the left side of the field (again standing behind the blue driver station)
 */
object EvergreenFieldConstants {
    val lengthY = Units.feetToMeters(26.0) + Units.inchesToMeters(11.25)
    val lengthX = Units.feetToMeters(54.0) + Units.inchesToMeters(3.25)
    val centerY = lengthY / 2.0
    val centerX = lengthX / 2.0

    fun Pose2d.isOnField() = (x in 0.0..lengthX) && (y in 0.0..lengthY)

    fun Pose3d.isOnField() = (x in 0.0..lengthX) && (y in 0.0..lengthY)
}
