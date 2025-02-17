package org.team9432.frc2025.robot.subsystems.coralrollers.dispenser

class Dispenser() {
    enum class Goal(private val setpointSupplier: () -> Double) {
        IDLE({ 0.0 })
    }

    var goal = Goal.IDLE
}
