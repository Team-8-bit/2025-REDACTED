package org.team9432.frc2025.robot.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d

object VisionConstants {
    // AprilTag layout
    var aprilTagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

    // Camera names, must match names configured on coprocessor
    var camera0Name: String = "camera_0"
    var camera1Name: String = "camera_1"

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    var robotToCamera0: Transform3d = Transform3d(0.2, 0.0, 0.2, Rotation3d(0.0, -0.4, 0.0))
    var robotToCamera1: Transform3d = Transform3d(-0.2, 0.0, 0.2, Rotation3d(0.0, -0.4, Math.PI))

    // Basic filtering thresholds
    var maxAmbiguity: Double = 0.3
    var maxZError: Double = 0.5

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    var linearStdDevBaseline: Double = 0.02 // Meters
    var angularStdDevBaseline: Double = 0.06 // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    var cameraStdDevFactors: DoubleArray =
        doubleArrayOf(
            1.0, // Camera 0
            1.0, // Camera 1
        )
}
