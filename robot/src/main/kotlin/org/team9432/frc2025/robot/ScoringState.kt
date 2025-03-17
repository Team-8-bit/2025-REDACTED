package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Pose2d
import org.littletonrobotics.junction.Logger

class ScoringState {
    var teleCoralTarget = CoralScoringTarget.L4
    var algaeTarget = AlgaeScoringTarget.PROCESSOR

    var autoCoralTarget: CoralScoringTarget? = null
    var autoBranchTarget: FieldConstants.Reef.Branch? = null
    var autoCoralStationPose: Pose2d? = null
    var climbMode = false

    fun clearAutoState() {
        autoCoralTarget = null
        autoBranchTarget = null
        autoCoralStationPose = null
    }

    val coralTarget
        get() = autoCoralTarget ?: teleCoralTarget

    fun log() {
        Logger.recordOutput("ScoringState/teleCoralTarget", teleCoralTarget)
        Logger.recordOutput("ScoringState/algaeTarget", algaeTarget)
        Logger.recordOutput("ScoringState/autoCoralTarget", autoCoralTarget)
        Logger.recordOutput("ScoringState/autoBranchTarget", autoBranchTarget)
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
