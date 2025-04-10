// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package org.team9432.frc2025.robot.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import org.team9432.frc2025.lib.AllianceTracker
import org.team9432.frc2025.lib.util.flip
import org.team9432.frc2025.robot.RobotPosition
import org.team9432.frc2025.robot.vision.VisionConstants

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses have a blue alliance
 * origin.
 */
object FieldConstants {
    val fieldLength: Double = VisionConstants.aprilTagLayout.fieldLength
    val fieldWidth: Double = VisionConstants.aprilTagLayout.fieldWidth
    //    val startingLineX: Double = Units.inchesToMeters(299.438) // Measured from the inside of
    // starting line

    //    val aprilTagWidth: Double = Units.inchesToMeters(6.50)
    const val APRIL_TAG_COUNT: Int = 22

    object Processor {
        val blueCenterFace =
            Pose2d(VisionConstants.aprilTagLayout.getTagPose(16).get().x, 0.0, Rotation2d.fromDegrees(90.0))
        val redCenterFace = blueCenterFace.flip()
    }

    object Barge {
        val netWidth: Double = Units.inchesToMeters(40.0)
        //        val netHeight: Double = Units.inchesToMeters(88.0)

        //        val farCage: Translation2d = Translation2d(Units.inchesToMeters(345.428),
        // Units.inchesToMeters(286.779))
        //        val middleCage: Translation2d = Translation2d(Units.inchesToMeters(345.428),
        // Units.inchesToMeters(242.855))
        //        val closeCage: Translation2d = Translation2d(Units.inchesToMeters(345.428),
        // Units.inchesToMeters(199.947))

        // Measured from floor to bottom of cage
        //        val deepHeight: Double = Units.inchesToMeters(3.125)
        //        val shallowHeight: Double = Units.inchesToMeters(30.125)
    }

    object CoralStation {
        private val BLUE_RIGHT =
            Pose2d(Units.inchesToMeters(33.526), Units.inchesToMeters(25.824), Rotation2d.fromDegrees(144.011 - 90))
        private val BLUE_LEFT =
            Pose2d(BLUE_RIGHT.x, fieldWidth - BLUE_RIGHT.y, Rotation2d.fromRadians(-BLUE_RIGHT.rotation.radians))
        private val BLUE_POSES = setOf(BLUE_RIGHT, BLUE_LEFT)

        private val RED_RIGHT = BLUE_RIGHT.flip()
        private val RED_LEFT = BLUE_LEFT.flip()
        private val RED_POSES = setOf(RED_RIGHT, RED_LEFT)

        val ALLIANCE_RIGHT
            get() = AllianceTracker.switch(blue = BLUE_RIGHT, red = RED_RIGHT)

        val ALLIANCE_LEFT
            get() = AllianceTracker.switch(blue = BLUE_LEFT, red = RED_LEFT)

        val ALLIANCE_POSES
            get() = AllianceTracker.switch(blue = BLUE_POSES, red = RED_POSES)
    }

    object Reef {
        //        val faceLength: Double = Units.inchesToMeters(36.792600)
        private val BLUE_CENTER = Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0)
        private val RED_CENTER = BLUE_CENTER.flip()
        //        val faceToZoneLine: Double = Units.inchesToMeters(12.0) // Side of the reef to the
        // inside of the reef zone line

        val ALLIANCE_CENTER
            get() = AllianceTracker.switch(blue = BLUE_CENTER, red = RED_CENTER)

        val maxRadius: Double = Units.inchesToMeters(76.0 / 2)
        val faceToCenter: Double = Units.inchesToMeters(65.491090 / 2)

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
                val poseDirection = Pose2d(BLUE_CENTER, Rotation2d.fromDegrees((180 + (60 * face)).toDouble()))
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

        enum class Branch(index: Int) {
            A(0),
            B(1),
            C(2),
            D(3),
            E(4),
            F(5),
            G(6),
            H(7),
            I(8),
            J(9),
            K(10),
            L(11);

            fun getAllianceTag(): Int {
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

            private val bluePose = branchPositions2d[index]
            private val redPose = bluePose.flip()

            private val alignPoseNormalBlue = bluePose.transformBy(RobotPosition.REEF_ALIGN_TRANSFORM)
            private val alignPoseNormalRed = redPose.transformBy(RobotPosition.REEF_ALIGN_TRANSFORM)

            private val alignPoseBlockedBlue = bluePose.transformBy(RobotPosition.REEF_ALIGN_BLOCKED_TRANSFORM)
            private val alignPoseBlockedRed = redPose.transformBy(RobotPosition.REEF_ALIGN_BLOCKED_TRANSFORM)

            val allianceNormalAlignPose: Pose2d
                get() = AllianceTracker.switch(blue = alignPoseNormalBlue, red = alignPoseNormalRed)

            val allianceBlockedAlignPose: Pose2d
                get() = AllianceTracker.switch(blue = alignPoseBlockedBlue, red = alignPoseBlockedRed)

            val alliancePose
                get() = AllianceTracker.switch(blue = bluePose, red = redPose)
        }

        enum class StagedAlgae(private val high: Boolean, centerFace: Int) {
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

            private val bluePose = centerFaces[centerFace]!!
            private val redPose = bluePose.flip()

            val alliancePose
                get() = AllianceTracker.switch(blue = bluePose, red = redPose)
        }
    }

    //    object StagingPositions {
    // Measured from the center of the ice cream
    //        val separation: Double = Units.inchesToMeters(72.0)
    //        val middleIceCream: Pose2d = Pose2d(Units.inchesToMeters(48.0), fieldWidth / 2.0,
    // Rotation2d())
    //        val leftIceCream: Pose2d = Pose2d(Units.inchesToMeters(48.0), middleIceCream.y +
    // separation, Rotation2d())
    //        val rightIceCream: Pose2d = Pose2d(Units.inchesToMeters(48.0), middleIceCream.y -
    // separation, Rotation2d())
    //    }
}
