package org.team9432.frc2025.robot

class ScoringState {
    var holdingCoral = false
    var holdingAlgae = false

    var target = ScoringTarget.L4
    var algaeIntakeTarget = AlgaeIntakeTarget.HIGH

    enum class ScoringTarget(val isCoral: Boolean) {
        L2(isCoral = true),
        L3(isCoral = true),
        L4(isCoral = true),
        PROCESSOR(isCoral = false),
        NET(isCoral = false);

        val isAlgae = !isCoral
    }

    enum class AlgaeIntakeTarget {
        LOW,
        HIGH,
    }
}
