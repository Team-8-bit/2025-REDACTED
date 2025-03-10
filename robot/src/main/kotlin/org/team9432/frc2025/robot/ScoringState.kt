package org.team9432.frc2025.robot

class ScoringState {
    var coralTarget = CoralScoringTarget.L4
    var algaeTarget = AlgaeScoringTarget.NET

    enum class CoralScoringTarget {
        L2,
        L3,
        L4,
    }

    enum class AlgaeScoringTarget {
        PROCESSOR,
        NET,
    }
}
