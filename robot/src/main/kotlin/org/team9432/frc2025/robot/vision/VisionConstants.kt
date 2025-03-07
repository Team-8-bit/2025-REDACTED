package org.team9432.frc2025.robot.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units

object VisionConstants {
    // AprilTag layout
    val aprilTagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)

    // Basic filtering thresholds
    const val MAX_AMBIGUITY = 0.4
    const val FIELD_BORDER_MARGIN = 0.4
    const val MAX_Z_ERROR = 0.3 // Meters
    const val MAX_ANGLE_ERROR = 5.0 // Tilt of the robot in degrees

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    const val LINEAR_STDDEV_BASELINE = 0.02 // Meters
    const val ANGULAR_STDDEV_BASELINE = 0.06 // Radians

    enum class CameraConstants(
        /** Standard deviation multiplier. Adjust to trust some cameras more than others. */
        val stdDevFactor: Double,
        /** Camera name, must match name configured on the coprocessor. */
        val cameraName: String,
        /** Robot to camera transform. */
        val robotToCamera: Transform3d,
    ) {
        FRONT(
            stdDevFactor = 1.0,
            cameraName = "FrontCamera",
            robotToCamera =
                Transform3d(
                    Translation3d(Units.inchesToMeters(13.0), Units.inchesToMeters(11.5), Units.inchesToMeters(8.5)),
                    Rotation3d(
                        Units.degreesToRadians(0.0),
                        Units.degreesToRadians(-20.0),
                        Units.degreesToRadians(-39.901730),
                    ),
                ),
        );

        val pose = Pose3d(robotToCamera.x, robotToCamera.y, robotToCamera.z, robotToCamera.rotation)
    }
}
