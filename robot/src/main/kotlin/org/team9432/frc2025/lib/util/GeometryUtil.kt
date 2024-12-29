package org.team9432.frc2025.lib.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team9432.frc2025.lib.AllianceTracker
import org.team9432.frc2025.lib.constants.EvergreenFieldConstants
import kotlin.math.atan2
import kotlin.math.hypot

/** Flips this [Pose2d] to the opposite side of a mirrored field. */
fun Pose2d.flip() = Pose2d(translation.flip(), rotation.flip())

/** Flips this [Translation2d] to the opposite side of a mirrored field. */
fun Translation2d.flip() = Translation2d(EvergreenFieldConstants.centerX + (EvergreenFieldConstants.centerX - x), y)

/** Flips this [Translation3d] to the opposite side of a mirrored field. */
fun Translation3d.flip() = Translation3d(EvergreenFieldConstants.centerX + (EvergreenFieldConstants.centerX - x), y, z)

/** Flips this [Rotation2d] to the opposite side of a mirrored field. */
fun Rotation2d.flip(): Rotation2d = Rotation2d.fromDegrees((degrees + 180) * -1)

/** Flips this [Pose2d] to the correct side of a mirrored field based on the current alliance color. */
fun Pose2d.applyFlip() = AllianceTracker.switch(blue = this, red = this.flip())

/** Flips this [Translation2d] to the correct side of a mirrored field based on the current alliance color. */
fun Translation2d.applyFlip() = AllianceTracker.switch(blue = this, red = this.flip())

/** Flips this [Translation3d] to the correct side of a mirrored field based on the current alliance color. */
fun Translation3d.applyFlip() = AllianceTracker.switch(blue = this, red = this.flip())

/** Flips this [Rotation2d] to the correct side of a mirrored field based on the current alliance color. */
fun Rotation2d.applyFlip() = AllianceTracker.switch(blue = this, red = this.flip())


/** Returns the angle this pose would need to be at to point at the given pose in radians. */
fun Translation2d.angleTo(pose: Translation2d) = atan2(pose.y - this.y, pose.x - this.x)

/** Returns the angle this pose would need to be at to point at the given pose in radians. */
fun Pose2d.angleTo(pose: Pose2d) = atan2(pose.y - this.y, pose.x - this.x)

/** Returns the angle this pose would need to be at to point at the given pose in radians. */
fun Pose2d.angleTo(pose: Translation2d) = atan2(pose.y - this.y, pose.x - this.x)

/** Returns true if this pose is within [epsilonMeters] of the given pose. */
fun Translation2d.isNear(pose: Translation2d, epsilonMeters: Double) = hypot(this.x - pose.x, this.y - pose.y) < epsilonMeters

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

/** Gets the position this pose would be in if it moved linearly at [speeds] for [timeSeconds]. */
fun Pose2d.transformBySpeeds(speeds: ChassisSpeeds, timeSeconds: Double) = Pose2d(
    this.x + speeds.vxMetersPerSecond * timeSeconds,
    this.y + speeds.vyMetersPerSecond * timeSeconds,
    Rotation2d(this.rotation.radians + speeds.omegaRadiansPerSecond * timeSeconds)
)