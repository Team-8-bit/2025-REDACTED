package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import edu.wpi.first.math.util.Units

object ElevatorConstants {
    const val REDUCTION = (42.0 / 14.0) * (46.0 / 20.0)
    val DRUM_RADIUS = Units.inchesToMeters(1.879783) / 2

    val MOTOR_ROTATIONS_PER_METER = REDUCTION / (DRUM_RADIUS * 2 * Math.PI)

    val POSITION_TOLERANCE = Units.inchesToMeters(0.5)
    val MIN_POSITION = Units.inchesToMeters(0.0)
    val MAX_POSITION = Units.inchesToMeters(53.0)

    const val PEAK_TORQUE_AMPS = 60.0
}
