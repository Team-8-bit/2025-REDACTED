package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Pose2d

class ScoringState {
    var teleCoralTarget = CoralScoringTarget.L4
    var algaeTarget = AlgaeScoringTarget.NET

    var autoCoralTarget: CoralScoringTarget? = null
    var autoBranchTarget: FieldConstants.Reef.Branch? = null
    var autoCoralStationPose: Pose2d? = null

    fun clearAutoState() {
        autoCoralTarget = null
        autoBranchTarget = null
        autoCoralStationPose = null
    }

    val coralTarget
        get() = autoCoralTarget ?: teleCoralTarget

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
