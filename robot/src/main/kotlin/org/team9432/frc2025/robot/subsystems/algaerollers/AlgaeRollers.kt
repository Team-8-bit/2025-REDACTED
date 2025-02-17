package org.team9432.frc2025.robot.subsystems.algaerollers

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class AlgaeRollers() : SubsystemBase() {
    enum class Goal(private val setpointSupplier: () -> Double) {
        IDLE({ 0.0 })
    }

    var goal = Goal.IDLE

    override fun periodic() {
        Logger.recordOutput("AlgaeRollers/Goal", goal)
    }
}
