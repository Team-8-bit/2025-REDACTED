package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.*
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure.State
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure.State.*
import org.team9432.frc2025.robot.subsystems.superstructure.arm.Arm
import org.team9432.frc2025.robot.subsystems.superstructure.dispenser.Dispenser
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

class TransitionCommands(private val elevator: Elevator, private val arm: Arm, private val dispenser: Dispenser) {
    private val transitions: Map<Pair<State, State>, Command>

    private fun setDispenser(goal: Dispenser.Goal) = runOnce({ dispenser.goal = goal })

    // Builds the set of legal moves
    init {
        // Rather have it less readable on small screens than not readable at all; spotless:off
        val transitions = mutableMapOf<Pair<State, State>, Command>()

        transitions[STOW to ARM_ABOVE_BUMPER] = elevator.runToGoal(Elevator.Goal.MIN_ARM_OUT)
        transitions[ARM_ABOVE_BUMPER to STOW] = elevator.runToGoal(Elevator.Goal.STOW)

        transitions[ARM_ABOVE_BUMPER to PREPARE_TALL_SCORE] = arm.runToGoal(Arm.Goal.PREPARE_SCORE)
        transitions[PREPARE_TALL_SCORE to ARM_ABOVE_BUMPER] = arm.runToGoal(Arm.Goal.STOW)

        transitions[STOW to INTAKE_CORAL] = setDispenser(Dispenser.Goal.INTAKE_CORAL)
        transitions[INTAKE_CORAL to STOW] = setDispenser(Dispenser.Goal.IDLE)

        transitions[PREPARE_TALL_SCORE to PREPARE_L2] = elevator.runToGoal(Elevator.Goal.L2).andThen(arm.runToGoal(Arm.Goal.L2))
        transitions[PREPARE_TALL_SCORE to PREPARE_L3] = elevator.runToGoal(Elevator.Goal.L3).andThen(arm.runToGoal(Arm.Goal.L3))
        transitions[PREPARE_TALL_SCORE to PREPARE_L4] = elevator.runToGoal(Elevator.Goal.L4).andThen(arm.runToGoal(Arm.Goal.L4))

        for (scoringGoal in setOf(PREPARE_L2, PREPARE_L3, PREPARE_L4)) {
            transitions[scoringGoal to PREPARE_TALL_SCORE] = arm.runToGoal(Arm.Goal.PREPARE_SCORE).andThen(elevator.runToGoal(Elevator.Goal.MIN_ARM_OUT))
        }

        transitions[PREPARE_L2 to SCORE_L2] = setDispenser(Dispenser.Goal.OUTTAKE_CORAL)
        transitions[SCORE_L2 to PREPARE_L2] = setDispenser(Dispenser.Goal.IDLE)
        transitions[PREPARE_L3 to SCORE_L3] = setDispenser(Dispenser.Goal.OUTTAKE_CORAL)
        transitions[SCORE_L3 to PREPARE_L3] = setDispenser(Dispenser.Goal.IDLE)
        transitions[PREPARE_L4 to SCORE_L4] = setDispenser(Dispenser.Goal.OUTTAKE_CORAL)
        transitions[SCORE_L4 to PREPARE_L4] = setDispenser(Dispenser.Goal.IDLE)

        transitions.forEach { (between, command) ->
            command.addRequirements(elevator, arm)
            command.name = "Transition from ${between.first} to ${between.second}."
        }

        transitions[ARM_ABOVE_BUMPER to INTAKE_ALGAE_LOW] =
            parallel(
                setDispenser(Dispenser.Goal.INTAKE_ALGAE),
                elevator.runToGoal(Elevator.Goal.INTAKE_ALGAE_REEF_LOW),
                arm.runToGoal(Arm.Goal.INTAKE_ALGAE_REEF)
            )
        transitions[ARM_ABOVE_BUMPER to INTAKE_ALGAE_HIGH] =
            parallel(
                setDispenser(Dispenser.Goal.INTAKE_ALGAE),
                elevator.runToGoal(Elevator.Goal.INTAKE_ALGAE_REEF_HIGH),
                arm.runToGoal(Arm.Goal.INTAKE_ALGAE_REEF)
            )

        transitions[INTAKE_ALGAE_LOW to HOLD_ALGAE_LOW] =
            sequence(
                setDispenser(Dispenser.Goal.HOLD_ALGAE),
                arm.runToGoal(Arm.Goal.HOLD_ALGAE_LOW),
                elevator.runToGoal(Elevator.Goal.HOLD_ALGAE_LOW)
            )
        transitions[INTAKE_ALGAE_HIGH to HOLD_ALGAE_LOW] =
            sequence(
                setDispenser(Dispenser.Goal.HOLD_ALGAE),
                arm.runToGoal(Arm.Goal.HOLD_ALGAE_LOW),
                elevator.runToGoal(Elevator.Goal.HOLD_ALGAE_LOW)
            )

        transitions[HOLD_ALGAE_LOW to PREPARE_PROCESSOR] = parallel(arm.runToGoal(Arm.Goal.PREPARE_PROCESSOR), elevator.runToGoal(Elevator.Goal.PREPARE_PROCESSOR))
        transitions[HOLD_ALGAE_LOW to PREPARE_NET] = parallel(arm.runToGoal(Arm.Goal.PREPARE_NET), elevator.runToGoal(Elevator.Goal.PREPARE_NET))

        transitions[PREPARE_PROCESSOR to HOLD_ALGAE_LOW] = parallel(arm.runToGoal(Arm.Goal.HOLD_ALGAE_LOW), elevator.runToGoal(Elevator.Goal.HOLD_ALGAE_LOW))
        transitions[PREPARE_NET to HOLD_ALGAE_LOW] = parallel(arm.runToGoal(Arm.Goal.HOLD_ALGAE_LOW), elevator.runToGoal(Elevator.Goal.HOLD_ALGAE_LOW))

        transitions[PREPARE_PROCESSOR to SCORE_PROCESSOR] = parallel(
            arm.runToGoal(Arm.Goal.SCORE_PROCESSOR),
            elevator.runToGoal(Elevator.Goal.SCORE_PROCESSOR)
        ).andThen(setDispenser(Dispenser.Goal.SCORE_ALGAE_PROCESSOR))
        transitions[PREPARE_NET to SCORE_NET] = parallel(
            arm.runToGoal(Arm.Goal.SCORE_NET),
            elevator.runToGoal(Elevator.Goal.SCORE_NET)
        ).andThen(setDispenser(Dispenser.Goal.SCORE_ALGAE_NET))

        transitions[SCORE_PROCESSOR to ARM_ABOVE_BUMPER] = parallel(
            setDispenser(Dispenser.Goal.IDLE),
            elevator.runToGoal(Elevator.Goal.MIN_ARM_OUT),
            arm.runToGoal(Arm.Goal.STOW)
        )
        transitions[SCORE_NET to ARM_ABOVE_BUMPER] = parallel(
            setDispenser(Dispenser.Goal.IDLE),
            elevator.runToGoal(Elevator.Goal.MIN_ARM_OUT),
            arm.runToGoal(Arm.Goal.STOW)
        )

        // yay for spotless in all the other places though. spotless:on

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
