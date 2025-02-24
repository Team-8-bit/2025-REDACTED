package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure.State
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure.State.*
import org.team9432.frc2025.robot.subsystems.superstructure.arm.Arm
import org.team9432.frc2025.robot.subsystems.superstructure.dispenser.Dispenser
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

class TransitionCommands(private val elevator: Elevator, private val arm: Arm, private val dispenser: Dispenser) {
    private val transitions: Map<Pair<State, State>, Command>

    private fun runDispenser(goal: Dispenser.Goal) = Commands.runOnce({ dispenser.goal = goal })

    // Builds the set of legal moves
    init {
        val transitions = mutableMapOf<Pair<State, State>, Command>()
        transitions[STOW to PREPARE_TALL_SCORE] =
            Commands.sequence(elevator.runToGoal(Elevator.Goal.MIN_ARM_OUT), arm.runToGoal(Arm.Goal.PREPARE_SCORE))

        transitions[PREPARE_TALL_SCORE to STOW] =
            Commands.sequence(arm.runToGoal(Arm.Goal.STOW), elevator.runToGoal(Elevator.Goal.STOW))

        transitions[STOW to INTAKE_CORAL] = runDispenser(Dispenser.Goal.INTAKE_CORAL)
        transitions[INTAKE_CORAL to STOW] = runDispenser(Dispenser.Goal.IDLE)

        transitions[PREPARE_TALL_SCORE to PREPARE_L2] =
            elevator.runToGoal(Elevator.Goal.L2).andThen(arm.runToGoal(Arm.Goal.L2))
        transitions[PREPARE_TALL_SCORE to PREPARE_L3] =
            elevator.runToGoal(Elevator.Goal.L3).andThen(arm.runToGoal(Arm.Goal.L3))
        transitions[PREPARE_TALL_SCORE to PREPARE_L4] =
            elevator.runToGoal(Elevator.Goal.L4).andThen(arm.runToGoal(Arm.Goal.L4))

        for (scoringGoal in setOf(PREPARE_L2, PREPARE_L3, PREPARE_L4)) {
            transitions[scoringGoal to PREPARE_TALL_SCORE] =
                arm.runToGoal(Arm.Goal.PREPARE_SCORE).andThen(elevator.runToGoal(Elevator.Goal.MIN_ARM_OUT))
        }

        transitions[PREPARE_L2 to SCORE_L2] = runDispenser(Dispenser.Goal.OUTTAKE_CORAL)
        transitions[SCORE_L2 to PREPARE_L2] = runDispenser(Dispenser.Goal.IDLE)
        transitions[PREPARE_L3 to SCORE_L3] = runDispenser(Dispenser.Goal.OUTTAKE_CORAL)
        transitions[SCORE_L3 to PREPARE_L3] = runDispenser(Dispenser.Goal.IDLE)
        transitions[PREPARE_L4 to SCORE_L4] = runDispenser(Dispenser.Goal.OUTTAKE_CORAL)
        transitions[SCORE_L4 to PREPARE_L4] = runDispenser(Dispenser.Goal.IDLE)

        transitions[STOW to TEST_ARM] = arm.runToGoal(Arm.Goal.TEST)
        transitions[TEST_ARM to STOW] = arm.runToGoal(Arm.Goal.STOW)

        transitions.forEach { (between, command) ->
            command.addRequirements(elevator, arm)
            command.name = "Transition from ${between.first} to ${between.second}."
        }

        this.transitions = transitions
    }

    operator fun get(between: Pair<State?, State?>): Command {
        val error = {
            """|Failed to fetch transition between ${between.first} and ${between.second}!
               |This should never happen, and there are unit tests to prevent this.
               |"""
                .trimMargin()
        }

        return transitions[between] ?: Commands.print(error.invoke())
    }

    val allValidMoves = transitions.keys
}
