package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import jdk.internal.loader.Loader
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

class Superstructure(private val elevator: Elevator) : SubsystemBase() {
    private var goal = Goal.IDLE

    enum class Goal {
        STOW,
        INTAKE_PICKUP,
        ELEVATOR_DIAGNOSTIC_MAX,
        ELEVATOR_DIAGNOSTIC_MID,
        ELEVATOR_DIAGNOSTIC_MIN,
    }

    init {
        defaultCommand = runGoal(Goal.STOW)
    }

    override fun periodic() {
        if (DriverStation.isDisabled()) {
            goal = Goal.STOW
        }

        elevator.goal = Intake.Goal.IDLE

        when (goal) {
            Goal.IDLE -> {}
            Goal.INTAKE -> {
                intake.goal = Intake.Goal.FLOOR_INTAKE
                loader.goal = Loader.Goal.FLOOR_INTAKE
            }

            Goal.SHOOTER_FEED -> {
                loader.goal = Loader.Goal.SHOOTER_FEED
            }

            Goal.FLOOR_EJECT -> {
                intake.goal = Intake.Goal.FLOOR_EJECT
                loader.goal = Loader.Goal.REVERSE
            }

            Goal.ALIGN_REVERSE -> {
                loader.goal = Loader.Goal.ALIGN_REVERSE
            }

            Goal.ALIGN_FORWARD -> {
                loader.goal = Loader.Goal.ALIGN_FORWARD
                intake.goal = Intake.Goal.LOAD
            }

            Goal.ALIGN_FORWARD_SLOW -> {
                loader.goal = Loader.Goal.ALIGN_FORWARD_SLOW
                intake.goal = Intake.Goal.LOAD
            }

            Goal.ALIGN_REVERSE_SLOW -> {
                loader.goal = Loader.Goal.ALIGN_REVERSE_SLOW
            }
        }

        intake.periodic()
        loader.periodic()

        Logger.recordOutput("Rollers/Goal", goal)
    }

    val isIntaking
        get() = goal == Goal.INTAKE

    val noteCurrentExceeded
        get() = intake.noteCurrentExceeded

    fun runGoal(newGoal: Goal): Command = startEnd({ goal = newGoal }, { goal = Goal.IDLE })
}
