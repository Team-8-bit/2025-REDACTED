package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

class CoralArm() {
    enum class Goal(private val setpointSupplier: () -> Double) {
        STOW({ 0.0 })
    }

    var goal = Goal.STOW
}
