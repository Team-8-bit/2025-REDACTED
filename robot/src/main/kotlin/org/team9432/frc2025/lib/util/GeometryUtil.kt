package org.team9432.frc2025.lib.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import kotlin.math.atan2
import kotlin.math.hypot
import org.team9432.frc2025.lib.AllianceTracker
import org.team9432.frc2025.robot.util.FieldConstants

fun applyX(x: Double): Double {
    return AllianceTracker.switch(blue = x, red = FieldConstants.fieldLength - x)
}

fun applyY(y: Double): Double {
    return AllianceTracker.switch(blue = y, red = FieldConstants.fieldWidth - y)
}

/** Flips this [Pose2d] to the opposite side of a rotated field. */
fun Pose2d.flip() = Pose2d(translation.flip(), rotation.flip())

/** Flips this [Translation2d] to the opposite side of a rotated field. */
fun Translation2d.flip() = Translation2d(FieldConstants.fieldLength - x, FieldConstants.fieldWidth - y)

/** Flips this [Translation3d] to the opposite side of a rotated field. */
fun Translation3d.flip() = Translation3d(FieldConstants.fieldLength - x, FieldConstants.fieldWidth - y, z)

/** Flips this [Rotation2d] to the opposite side of a rotated field. */
fun Rotation2d.flip(): Rotation2d = this.rotateBy(Rotation2d.k180deg)

/** Flips this [Pose2d] to the correct side of a rotated field based on the current alliance color. */
fun Pose2d.applyFlip() = AllianceTracker.switch(blue = this, red = this.flip())

/** Flips this [Translation2d] to the correct side of a rotated field based on the current alliance color. */
fun Translation2d.applyFlip() = AllianceTracker.switch(blue = this, red = this.flip())

/** Flips this [Translation3d] to the correct side of a rotated field based on the current alliance color. */
fun Translation3d.applyFlip() = AllianceTracker.switch(blue = this, red = this.flip())

/** Flips this [Rotation2d] to the correct side of a rotated field based on the current alliance color. */
fun Rotation2d.applyFlip() = AllianceTracker.switch(blue = this, red = this.flip())

/** Returns the angle this pose would need to be at to point at the given pose in radians. */
fun Translation2d.angleTo(pose: Translation2d) = atan2(pose.y - this.y, pose.x - this.x)

/** Returns the angle this pose would need to be at to point at the given pose in radians. */
fun Pose2d.angleTo(pose: Pose2d) = atan2(pose.y - this.y, pose.x - this.x)

/** Returns the angle this pose would need to be at to point at the given pose in radians. */
fun Pose2d.angleTo(pose: Translation2d) = atan2(pose.y - this.y, pose.x - this.x)

/** Returns true if this pose is within [epsilonMeters] of the given pose. */
fun Translation2d.isNear(pose: Translation2d, epsilonMeters: Double) =
    hypot(this.x - pose.x, this.y - pose.y) < epsilonMeters

/** Returns true if this pose is within [epsilonMeters] of the given pose. */
fun Pose2d.isNear(pose: Pose2d, epsilonMeters: Double) = hypot(this.x - pose.x, this.y - pose.y) < epsilonMeters

/** Return the distance from this pose to another in meters. */
fun Translation2d.distanceTo(pose: Translation2d) = this.getDistance(pose)

/** Return the distance from this pose to another in meters. */
fun Pose2d.distanceTo(pose: Pose2d) = this.translation.getDistance(pose.translation)

/** Return the distance from this pose to another in meters. */
fun Pose2d.distanceTo(pose: Translation2d) = this.translation.getDistance(pose)

/** Returns a Pose2d at this translation with rotation to point at the given pose. */
fun Translation2d.pointAt(pose: Translation2d) = Pose2d(x, y, Rotation2d(this.angleTo(pose)))

/** Returns a Pose2d at this translation with rotation to point at the given pose. */
fun Translation2d.pointAt(pose: Pose2d) = Pose2d(x, y, Rotation2d(this.angleTo(pose.translation)))

/** Returns the norm, or distance from the origin to the translation on the xy plane. */
fun Translation3d.xyNorm() = hypot(x, y)

/** Gets the position this pose would be in if it moved linearly at [speeds] for [timeSeconds]. */
fun Pose2d.transformBySpeeds(speeds: ChassisSpeeds, timeSeconds: Double) =
    Pose2d(
        this.x + speeds.vxMetersPerSecond * timeSeconds,
        this.y + speeds.vyMetersPerSecond * timeSeconds,
        Rotation2d(this.rotation.radians + speeds.omegaRadiansPerSecond * timeSeconds),
    )

fun Rotation2dWithout0Error(x: Double, y: Double): Rotation2d {
    val cos: Double
    val sin: Double

    val magnitude = hypot(x, y)
    if (magnitude > 1e-6) {
        cos = x / magnitude
        sin = y / magnitude
    } else {
        cos = 1.0
        sin = 0.0
    }
    return Rotation2d(atan2(sin, cos))
}
