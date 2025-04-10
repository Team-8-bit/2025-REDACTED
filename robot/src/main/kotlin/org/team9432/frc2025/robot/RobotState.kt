package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.util.FieldConstants

class RobotState {
    var teleCoralTarget = CoralScoringTarget.L4
    var teleBranchTarget: FieldConstants.Reef.Branch = FieldConstants.Reef.Branch.A
    var algaeTarget = AlgaeScoringTarget.PROCESSOR
        get() {
            return if (DriverStation.isAutonomousEnabled()) AlgaeScoringTarget.NET else field
        }

    var autoCoralTarget: CoralScoringTarget? = null
    var autoBranchTarget: FieldConstants.Reef.Branch? = null
    var autoCoralStationPose: Pose2d? = null

    var autoAlgaePickupTarget: FieldConstants.Reef.StagedAlgae? = null

    fun clearAutoState() {
        autoCoralTarget = null
        autoBranchTarget = null
        autoCoralStationPose = null
        autoAlgaePickupTarget = null
    }

    val coralTarget
        get() = autoCoralTarget ?: teleCoralTarget

    val branchTarget
        get() = autoBranchTarget ?: teleBranchTarget

    fun log() {
        Logger.recordOutput("RobotState/teleCoralTarget", teleCoralTarget)
        Logger.recordOutput("RobotState/algaeTarget", algaeTarget)
        Logger.recordOutput("RobotState/autoCoralTarget", autoCoralTarget)
        Logger.recordOutput("RobotState/autoBranchTarget", autoBranchTarget)
        Logger.recordOutput("RobotState/autoAlgaePickupTarget", autoAlgaePickupTarget?.name ?: "null")
        Logger.recordOutput("RobotState/autoCoralStationPose", autoCoralStationPose ?: Pose2d())
    }

    enum class CoralScoringTarget {
        L1,
        L2,
        L3,
        L4;

        fun isL1() = this == L1

        fun isL2() = this == L2

        fun isL3() = this == L3

        fun isL4() = this == L4

        fun isL2OrL3() = this == L2 || this == L3
    }

    enum class AlgaeScoringTarget {
        PROCESSOR,
        NET,
    }
}
