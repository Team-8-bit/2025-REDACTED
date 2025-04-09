package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import org.team9432.frc2025.robot.subsystems.superstructure.SuperstructureState.*
import org.team9432.frc2025.robot.subsystems.superstructure.arm.Arm
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

class TransitionCommands(private val elevator: Elevator, private val arm: Arm) {
    private val transitions: Map<Pair<SuperstructureState, SuperstructureState>, Command>

    // Builds the set of legal moves
    init {
        // Rather have it less readable on small screens than not readable at all; spotless:off
        val transitions = mutableMapOf<Pair<SuperstructureState, SuperstructureState>, Command>()

        fun addParallel(between: Pair<SuperstructureState, SuperstructureState>) {
            transitions[between] = parallel(elevator.runToGoal(between.second.elevatorGoal), arm.runToGoal(between.second.armGoal))
        }
        fun addElevatorFirst(between: Pair<SuperstructureState, SuperstructureState>) {
            transitions[between] = elevator.runToGoal(between.second.elevatorGoal).andThen(arm.runToGoal(between.second.armGoal))
        }
        fun addArmFirst(between: Pair<SuperstructureState, SuperstructureState>) {
            transitions[between] = arm.runToGoal(between.second.armGoal).andThen(elevator.runToGoal(between.second.elevatorGoal))
        }
        fun addSymmetricParallel(first: SuperstructureState, second: SuperstructureState) {
            addParallel(first to second)
            addParallel(second to first)
        }

        addElevatorFirst(STOW to ARM_ABOVE_BUMPER)
        addElevatorFirst(ARM_ABOVE_BUMPER to STOW)

        addArmFirst(ARM_ABOVE_BUMPER to SCORE_L1)
        addElevatorFirst(SCORE_L1 to ARM_ABOVE_BUMPER)

        addElevatorFirst(PREP_L4 to PLACE_L4)
        addArmFirst(PLACE_L4 to PREP_L4)
        addSymmetricParallel(PLACE_L4, SCORE_L4)

        val tallCoralGoals = setOf(UNJAM_CORAL, SCORE_L1, SCORE_L2, SCORE_L3, SCORE_L4, PREP_L4)
        for (tallCoralGoal in tallCoralGoals) {
            addSymmetricParallel(ARM_ABOVE_BUMPER, tallCoralGoal)

            addSymmetricParallel(tallCoralGoal, INTAKE_ALGAE_HIGH)
            addSymmetricParallel(tallCoralGoal, INTAKE_ALGAE_LOW)

            for (other in tallCoralGoals) {
                if (tallCoralGoal == other) continue

                addParallel(tallCoralGoal to other)
            }
        }

        addSymmetricParallel(ALGAE_STOW, PROCESSOR)
        addSymmetricParallel(ALGAE_STOW, PREP_NET)

        addElevatorFirst(ALGAE_STOW to ARM_ABOVE_BUMPER)

        addSymmetricParallel(PREP_NET, SCORE_NET)

        addSymmetricParallel(SCORE_NET, ARM_ABOVE_BUMPER)

        addSymmetricParallel(ARM_ABOVE_BUMPER, UNJAM_CORAL)

        for (algaeIntakeGoal in setOf(INTAKE_ALGAE_HIGH, INTAKE_ALGAE_LOW)) {
            addSymmetricParallel(algaeIntakeGoal, ARM_ABOVE_BUMPER)
            addSymmetricParallel(algaeIntakeGoal, ALGAE_STOW)
            addSymmetricParallel(algaeIntakeGoal, PREP_NET)
            addSymmetricParallel(algaeIntakeGoal, SCORE_NET)
        }

        addSymmetricParallel(INTAKE_ALGAE_LOW, INTAKE_ALGAE_HIGH)

        addParallel(PREP_NET to ARM_ABOVE_BUMPER)

        addArmFirst(ARM_ABOVE_BUMPER to ALGAE_FLOOR)
        addElevatorFirst(ALGAE_FLOOR to ARM_ABOVE_BUMPER)
        addParallel(ALGAE_FLOOR to ALGAE_STOW)
        addParallel(ALGAE_STOW to ALGAE_FLOOR)

        // yay for spotless in all the other places though. spotless:on

        transitions.forEach { (between, command) ->
            command.addRequirements(elevator, arm)
            command.name = "Transition from ${between.first} to ${between.second}."
        }

        this.transitions = transitions
    }

    operator fun get(between: Pair<SuperstructureState?, SuperstructureState?>): Command {
        val error = {
            """|Failed to fetch transition between ${between.first} and ${between.second}!
               |This should never happen, and there are unit tests to prevent this.
               |"""
                .trimMargin()
        }

        return transitions[between] ?: print(error.invoke())
    }

    val allValidMoves = transitions.keys
}
