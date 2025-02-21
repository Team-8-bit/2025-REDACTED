package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

class CoralArm() {
    enum class Goal(private val setpointSupplier: () -> Double) {
        STOW({ 0.0 }),
        PREPARE_SCORE({ 0.0 }),
    }

    fun atGoal(): Boolean = false

    var goal = Goal.STOW
}
