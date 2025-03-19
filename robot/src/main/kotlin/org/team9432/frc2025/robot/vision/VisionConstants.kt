package org.team9432.frc2025.robot.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

object VisionConstants {
    // AprilTag layout
    val aprilTagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)

    // Basic filtering thresholds
    const val MAX_AMBIGUITY = 0.4
    const val FIELD_BORDER_MARGIN = 0.4
    const val MAX_Z_ERROR = 0.3 // Meters
    const val MAX_ANGLE_ERROR = 5.0 // Tilt of the robot in degrees

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    const val LINEAR_STDDEV_BASELINE = 0.08 // Meters
    const val ANGULAR_STDDEV_BASELINE = 5.0 // Degrees

    enum class CameraConstants(
        /** Standard deviation multiplier. Adjust to trust some cameras more than others. */
        val stdDevFactor: Double,
        /** Camera name, must match name configured on the coprocessor. */
        val cameraName: String,
        /** Robot to camera transform. */
        private val initialRobotToCamera: Transform3d,
        /** Pitch offset to apply. */
        private val initialPitchOffset: Double,
    ) {
        FRONT_LEFT(
            stdDevFactor = 1.0,
            cameraName = "FrontLeft",
            initialRobotToCamera =
                Transform3d(
                    Translation3d(Units.inchesToMeters(11.0), Units.inchesToMeters(11.5), Units.inchesToMeters(8.5)),
                    Rotation3d(
                        Units.degreesToRadians(0.0),
                        Units.degreesToRadians(-20.0),
                        Units.degreesToRadians(-45.0),
                    ),
                ),
            initialPitchOffset = 0.0,
        ),
        FRONT_RIGHT(
            stdDevFactor = 1.0,
            cameraName = "FrontRight",
            initialRobotToCamera =
                Transform3d(
                    Translation3d(Units.inchesToMeters(11.0), Units.inchesToMeters(-11.5), Units.inchesToMeters(8.5)),
                    Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-20.0), Units.degreesToRadians(45.0)),
                ),
            initialPitchOffset = 0.0,
        );

        private val pitchOffset = LoggedTunableNumber("Vision/Constants/${cameraName}PitchOffset", initialPitchOffset)
        private var lastPitchOffset = initialPitchOffset

        private var currentRobotToCamera = initialRobotToCamera

        val robotToCamera: Transform3d
            get() {
                val pitchOffset = pitchOffset.get()
                if (pitchOffset != lastPitchOffset) {
                    currentRobotToCamera =
                        initialRobotToCamera.plus(
                            Transform3d(Translation3d.kZero, Rotation3d(0.0, Units.degreesToRadians(pitchOffset), 0.0))
                        )
                    lastPitchOffset = pitchOffset
                    return currentRobotToCamera
                } else {
                    return currentRobotToCamera
                }
            }
    }
}
