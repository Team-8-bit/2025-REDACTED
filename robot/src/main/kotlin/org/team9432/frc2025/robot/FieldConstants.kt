// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import org.team9432.frc2025.lib.AllianceTracker
import org.team9432.frc2025.robot.vision.VisionConstants

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses have a blue alliance
 * origin.
 */
object FieldConstants {
    val fieldLength: Double = VisionConstants.aprilTagLayout.fieldLength
    val fieldWidth: Double = VisionConstants.aprilTagLayout.fieldWidth
    val startingLineX: Double = Units.inchesToMeters(299.438) // Measured from the inside of starting line
    val algaeDiameter: Double = Units.inchesToMeters(16.0)

    val aprilTagWidth: Double = Units.inchesToMeters(6.50)
    const val APRIL_TAG_COUNT: Int = 22

    object Processor {
        val centerFace: Pose2d =
            Pose2d(VisionConstants.aprilTagLayout.getTagPose(16).get().x, 0.0, Rotation2d.fromDegrees(90.0))
    }

    object Barge {
        val netWidth: Double = Units.inchesToMeters(40.0)
        val netHeight: Double = Units.inchesToMeters(88.0)

        val farCage: Translation2d = Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779))
        val middleCage: Translation2d = Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855))
        val closeCage: Translation2d = Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947))

        // Measured from floor to bottom of cage
        val deepHeight: Double = Units.inchesToMeters(3.125)
        val shallowHeight: Double = Units.inchesToMeters(30.125)
    }

    enum class CoralStation(val centerPose: Pose2d) {
        RIGHT(Pose2d(Units.inchesToMeters(33.526), Units.inchesToMeters(25.824), Rotation2d.fromDegrees(144.011 - 90))),
        LEFT(
            Pose2d(
                RIGHT.centerPose.x,
                fieldWidth - RIGHT.centerPose.y,
                Rotation2d.fromRadians(-RIGHT.centerPose.rotation.radians),
            )
        );

        companion object {
            val stationLength: Double = Units.inchesToMeters(79.750)
        }
    }

    object Reef {
        val faceLength: Double = Units.inchesToMeters(36.792600)
        val center: Translation2d = Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0)
        val faceToZoneLine: Double = Units.inchesToMeters(12.0) // Side of the reef to the inside of the reef zone line

        val maxRadius: Double = Units.inchesToMeters(76.0 / 2)

        private val centerFaces: Array<Pose2d?> =
            arrayOfNulls(6) // Starting facing the driver station in counterclockwise order
        private val branchPositions2d: MutableList<Pose2d> = mutableListOf()

        init {
            // Initialize faces
            val aprilTagLayout = VisionConstants.aprilTagLayout
            centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d()
            centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d()
            centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d()
            centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d()
            centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d()
            centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d()

            // Initialize branch positions
            for (face in 0..5) {
                val poseDirection = Pose2d(center, Rotation2d.fromDegrees((180 + (60 * face)).toDouble()))
                val adjustX = Units.inchesToMeters(65.491090 / 2)
                val adjustY = Units.inchesToMeters(6.469)

                val leftBranchPose =
                    Pose2d(
                        Translation2d(
                            poseDirection.transformBy(Transform2d(adjustX, -adjustY, Rotation2d())).x,
                            poseDirection.transformBy(Transform2d(adjustX, -adjustY, Rotation2d())).y,
                        ),
                        poseDirection.rotation,
                    )
                val rightBranchPose =
                    Pose2d(
                        Translation2d(
                            poseDirection.transformBy(Transform2d(adjustX, adjustY, Rotation2d())).x,
                            poseDirection.transformBy(Transform2d(adjustX, adjustY, Rotation2d())).y,
                        ),
                        poseDirection.rotation,
                    )

                branchPositions2d.add(leftBranchPose)
                branchPositions2d.add(rightBranchPose)
            }
        }

        enum class Branch {
            A,
            B,
            C,
            D,
            E,
            F,
            G,
            H,
            I,
            J,
            K,
            L;

            fun getTag(): Int {
                return when (this) {
                    A,
                    B -> AllianceTracker.switch(blue = 18, red = 7)

                    C,
                    D -> AllianceTracker.switch(blue = 17, red = 8)

                    E,
                    F -> AllianceTracker.switch(blue = 22, red = 9)

                    G,
                    H -> AllianceTracker.switch(blue = 21, red = 10)

                    I,
                    J -> AllianceTracker.switch(blue = 20, red = 11)

                    K,
                    L -> AllianceTracker.switch(blue = 19, red = 6)
                }
            }

            fun getPose() = branchPositions2d[entries.indexOf(this)]

            val mirror
                get() =
                    when (this) {
                        A -> B
                        B -> A
                        L -> C
                        C -> L
                        K -> D
                        D -> K
                        J -> E
                        E -> J
                        I -> F
                        F -> I
                        H -> G
                        G -> H
                    }
        }

        enum class StagedAlgae(private val high: Boolean, val centerFace: Int) {
            AB(high = true, 0),
            CD(high = false, 5),
            EF(high = true, 4),
            GH(high = false, 3),
            IJ(high = true, 2),
            KL(high = false, 1);

            val isHigh
                get() = high

            val isLow
                get() = !high

            fun getTag() =
                when (this) {
                    AB -> AllianceTracker.switch(blue = 18, red = 7)
                    CD -> AllianceTracker.switch(blue = 17, red = 8)
                    EF -> AllianceTracker.switch(blue = 22, red = 9)
                    GH -> AllianceTracker.switch(blue = 21, red = 10)
                    IJ -> AllianceTracker.switch(blue = 20, red = 11)
                    KL -> AllianceTracker.switch(blue = 19, red = 6)
                }

            fun getPose() = centerFaces[centerFace]!!
        }
    }

    object StagingPositions {
        // Measured from the center of the ice cream
        val separation: Double = Units.inchesToMeters(72.0)
        val middleIceCream: Pose2d = Pose2d(Units.inchesToMeters(48.0), fieldWidth / 2.0, Rotation2d())
        val leftIceCream: Pose2d = Pose2d(Units.inchesToMeters(48.0), middleIceCream.y + separation, Rotation2d())
        val rightIceCream: Pose2d = Pose2d(Units.inchesToMeters(48.0), middleIceCream.y - separation, Rotation2d())
    }
}
