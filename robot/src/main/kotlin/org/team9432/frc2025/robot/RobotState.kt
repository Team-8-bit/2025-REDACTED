package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.util.FieldConstants

class RobotState {
    var teleCoralTarget = CoralScoringTarget.L4
    var algaeTarget = AlgaeScoringTarget.PROCESSOR
        get() {
            return if (DriverStation.isAutonomousEnabled()) AlgaeScoringTarget.NET else field
        }

    var autoCoralTarget: CoralScoringTarget? = null
    var autoBranchTarget: FieldConstants.Reef.Branch? = null
    var autoCoralStationPose: Pose2d? = null

    var autoAlgaePickupTarget: FieldConstants.Reef.StagedAlgae? = null

    var flipBranch = false

    fun clearAutoState() {
        autoCoralTarget = null
        autoBranchTarget = null
        autoCoralStationPose = null
        autoAlgaePickupTarget = null
    }

    val coralTarget
        get() = autoCoralTarget ?: teleCoralTarget

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
        L4,
    }

    enum class AlgaeScoringTarget {
        PROCESSOR,
        NET,
    }
}
