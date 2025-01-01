package org.team9432.frc2025.robot.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units

object VisionConstants {
    // Set to true to invalidate all vision readings
    const val DISABLED = false

    // AprilTag layout
    val aprilTagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

    // Basic filtering thresholds
    const val MAX_AMBIGUITY = 0.3
    const val MAX_Z_ERROR = 0.3 // Meters
    const val MAX_ANGLE_ERROR = 5.0 // Tilt of the robot in degrees

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    const val LINEAR_STDDEV_BASELINE = 0.02 // Meters
    const val ANGULAR_STDDEV_BASELINE = 0.06 // Radians

    enum class CameraConstants(
        /** Camera name to use in logs. */
        val logName: String,
        /** Standard deviation multiplier. Adjust to trust some cameras more than others. */
        val stddevFactor: Double,
    ) {
        FRONT(logName = "FrontCamera", stddevFactor = 1.0),
        BACK(logName = "BackCamera", stddevFactor = 1.0),
    }

    enum class PhotonConfig(
        /** Camera name as configured in photonvision, must match name configured on the coprocessor. */
        val photonName: String,
        /** Robot to camera transform. */
        val robotToCamera: Transform3d,
    ) {
        FRONT(
            photonName = "Front",
            robotToCamera =
                Transform3d(
                    Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.3)),
                    Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-15.0), Units.degreesToRadians(0.0)),
                ),
        ),
        BACK(
            photonName = "Back",
            robotToCamera =
                Transform3d(
                    Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.3)),
                    Rotation3d(
                        Units.degreesToRadians(0.0),
                        Units.degreesToRadians(-15.0),
                        Units.degreesToRadians(180.0),
                    ),
                ),
        ),
    }
}
