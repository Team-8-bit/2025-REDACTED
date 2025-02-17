package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import edu.wpi.first.math.util.Units
import org.team9432.frc2025.robot.Constants
import org.team9432.frc2025.robot.Constants.RobotType.COMP
import org.team9432.frc2025.robot.Constants.RobotType.SIM

object ElevatorConstants {
    const val REDUCTION = (42.0 / 14.0) * (46.0 / 20.0)
    val DRUM_RADIUS = Units.inchesToMeters(1.879783) / 2

    val MOTOR_ROTATIONS_PER_METER = REDUCTION / (DRUM_RADIUS * 2 * Math.PI)

    val POSITION_TOLERANCE = Units.inchesToMeters(0.5)
    val MIN_POSITION = Units.inchesToMeters(0.0)
    val MAX_POSITION = Units.inchesToMeters(53.0)

    const val PEAK_TORQUE_AMPS = 60.0

    val gains =
        when (Constants.robot) {
            COMP ->
                Gains(
                    kP = 0.0,
                    kI = 0.0,
                    kD = 0.0,
                    ffkS = 8.0,
                    ffkV = 0.0,
                    ffkA = 0.0,
                    ffkG = 0.0,
                    mmCruise = 0.0,
                    mmAccel = 0.0,
                    mmJerk = 0.0,
                )

            SIM ->
                Gains(
                    kP = 500.0,
                    kI = 0.0,
                    kD = 120.0,
                    ffkS = 0.0,
                    ffkV = 0.0,
                    ffkA = 0.0,
                    ffkG = 9.9,
                    mmCruise = 2.0,
                    mmAccel = 8.0,
                    mmJerk = 0.0,
                )
        }

    data class Gains(
        val kP: Double,
        val kI: Double,
        val kD: Double,
        val ffkS: Double,
        val ffkV: Double,
        val ffkA: Double,
        val ffkG: Double,
        val mmCruise: Double,
        val mmAccel: Double,
        val mmJerk: Double,
    )
}
