package org.team9432.frc2025.robot.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields

object VisionConstants {
    // AprilTag layout
    val aprilTagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

    // Basic filtering thresholds
    const val MAX_AMBIGUITY = 0.3
    const val MAX_Z_ERROR = 0.5

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    const val LINEAR_STDDEV_BASELINE = 0.02 // Meters
    const val ANGULAR_STDDEV_BASELINE = 0.06 // Radians
}
