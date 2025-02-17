package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

import org.team9432.frc2025.robot.Constants
import org.team9432.frc2025.robot.Constants.RobotType.COMP
import org.team9432.frc2025.robot.Constants.RobotType.SIM

object CoralArmConstants {
    const val REDUCTION = (5.0 / 1.0) * (3.0 / 1.0) * (42.0 / 12.0)

    // Rotations
    val POSITION_TOLERANCE = 0.05
    val MIN_POSITION = 0.0
    val MAX_POSITION = 0.5

    val gains =
        when (Constants.robot) {
            COMP ->
                Gains(
                    kP = 0.0,
                    kI = 0.0,
                    kD = 0.0,
                    ffkS = 0.0,
                    ffkV = 0.0,
                    ffkA = 0.0,
                    ffkG = 0.0,
                    mmCruise = 0.0,
                    mmAccel = 0.0,
                    mmJerk = 0.0,
                )

            SIM ->
                Gains(
                    kP = 0.0,
                    kI = 0.0,
                    kD = 0.0,
                    ffkS = 0.0,
                    ffkV = 0.0,
                    ffkA = 0.0,
                    ffkG = 0.0,
                    mmCruise = 0.0,
                    mmAccel = 0.0,
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
