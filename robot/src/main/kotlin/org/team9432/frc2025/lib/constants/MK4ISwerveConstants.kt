package org.team9432.frc2025.lib.constants

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import kotlin.math.hypot

/** Various swerve constants. */
@Suppress("unused")
object MK4ISwerveConstants {
    const val L1_DRIVE_REDUCTION = 8.14
    const val L2_DRIVE_REDUCTION = 6.75
    const val L3_DRIVE_REDUCTION = 6.12

    const val L1PLUS_DRIVE_REDUCTION = 7.13
    const val L2PLUS_DRIVE_REDUCTION = 5.90
    const val L3PLUS_DRIVE_REDUCTION = 5.36

    const val STEER_REDUCTION = 21.43

    /**
     * Returns an array of [Translation2d] of Mk4i swerve module positions given a frame size. Useful for WPILib
     * odometry classes.
     */
    fun getModuleTranslationsForFrameSize(frameSizeInches: Double): Array<Translation2d> {
        val distanceFromCenterXY = (frameSizeInches / 2) - 2.625
        val distanceInches = hypot(distanceFromCenterXY, distanceFromCenterXY)
        val moduleDistance = Units.inchesToMeters(distanceInches)

        val frontLeft = Translation2d(moduleDistance, moduleDistance)
        val frontRight = Translation2d(moduleDistance, -moduleDistance)
        val backLeft = Translation2d(-moduleDistance, moduleDistance)
        val backRight = Translation2d(-moduleDistance, -moduleDistance)
        return arrayOf(frontLeft, frontRight, backLeft, backRight)
    }
}
