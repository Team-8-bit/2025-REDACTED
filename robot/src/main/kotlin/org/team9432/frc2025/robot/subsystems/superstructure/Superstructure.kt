package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

class Superstructure(private val elevator: Elevator) : SubsystemBase() {
    private var goal = Goal.STOW

    enum class Goal {
        STOW,
        TEST_ELEVATOR,
    }

    init {
        defaultCommand = runGoal(Goal.STOW)
    }

    override fun periodic() {
        if (DriverStation.isDisabled()) {
            goal = Goal.STOW
        }

        when (goal) {
            Goal.STOW -> {
                elevator.goal = Elevator.Goal.STOW
            }
            Goal.TEST_ELEVATOR -> {
                elevator.goal = Elevator.Goal.TEST
            }
        }

        elevator.periodic()

        Logger.recordOutput("Rollers/Goal", goal)
    }

    fun runGoal(newGoal: Goal): Command = startEnd({ goal = newGoal }, { goal = Goal.STOW })
}
