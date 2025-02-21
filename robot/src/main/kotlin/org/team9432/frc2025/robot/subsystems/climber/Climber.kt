package org.team9432.frc2025.robot.subsystems.climber

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Climber() : SubsystemBase() {
    enum class Goal(private val setpointSupplier: () -> Double) {
        STOW({ 0.0 })
    }

    var goal = Goal.STOW

    override fun periodic() {
        Logger.recordOutput("Climber/Goal", goal)
    }
}
