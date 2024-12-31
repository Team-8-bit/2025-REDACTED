package org.team9432.frc2025.robot.vision

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units

enum class CameraConfig(
    /** Camera name, must match name configured on the coprocessor. */
    val cameraName: String,
    /** Robot to camera transform. */
    val robotToCamera: Transform3d,
    /** Standard deviation multiplier. Adjust to trust some cameras more than others. */
    val stddevFactor: Double,
) {
    FRONT(
        cameraName = "FrontCamera",
        robotToCamera =
            Transform3d(
                Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
                Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)),
            ),
        stddevFactor = 0.0,
    ),
    BACK(
        cameraName = "BackCamera",
        robotToCamera =
            Transform3d(
                Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
                Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)),
            ),
        stddevFactor = 0.0,
    ),
}
