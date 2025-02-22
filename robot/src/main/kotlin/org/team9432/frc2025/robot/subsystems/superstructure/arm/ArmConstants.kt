package org.team9432.frc2025.robot.subsystems.superstructure.arm

import edu.wpi.first.math.util.Units
import kotlin.math.acos
import kotlin.math.hypot

object ArmConstants {
    const val REDUCTION = (5.0 / 1.0) * (5.0 / 1.0) * (42.0 / 12.0)

    // Rotations
    val POSITION_TOLERANCE = Units.degreesToRotations(2.0)

    val MIN_POSITION = -0.25 // calculateAngle(2.136103, 7.911385) // -66.49438810123185 degrees
    val MAX_POSITION = 0.2 // calculateAngle(2.56658, 12.79353) // 78.656164156599 degrees

    private fun calculateAngle(x: Double, y: Double): Double {
        return Units.radiansToRotations(acos(x / hypot(x, y)))
    }

    const val PEAK_TORQUE_AMPS = 60.0
}
