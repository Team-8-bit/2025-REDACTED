package org.team9432.frc2025.robot.subsystems.coralrollers

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.subsystems.coralrollers.dispenser.Dispenser
import org.team9432.frc2025.robot.subsystems.coralrollers.funnel.Funnel

class CoralRollers(private val funnel: Funnel, private val dispenser: Dispenser) : SubsystemBase() {
    private var goal = Goal.IDLE

    enum class Goal {
        IDLE
    }

    init {
        defaultCommand = runGoal(Goal.IDLE)
    }

    override fun periodic() {
        if (DriverStation.isDisabled()) {
            goal = Goal.IDLE
        }

        Logger.recordOutput("CoralRollers/Goal", goal)
    }

    fun runGoal(newGoal: Goal): Command = startEnd({ goal = newGoal }, { goal = Goal.IDLE })
}
