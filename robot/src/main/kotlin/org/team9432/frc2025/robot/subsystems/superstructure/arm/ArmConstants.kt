package org.team9432.frc2025.robot.subsystems.superstructure.arm

object ArmConstants {
    const val REDUCTION = (4.0 / 1.0) * (4.0 / 1.0) * (42.0 / 12.0)

    // Rotations
    const val POSITION_TOLERANCE = 0.01

    const val MIN_POSITION = -0.25
    const val MAX_POSITION = 0.175

    const val PEAK_TORQUE_AMPS = 60.0
}
