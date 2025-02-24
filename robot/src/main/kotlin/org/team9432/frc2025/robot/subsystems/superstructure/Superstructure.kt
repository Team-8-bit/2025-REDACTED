package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.collections.set
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.util.chainAddRequirements
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure.State.STOW
import org.team9432.frc2025.robot.subsystems.superstructure.arm.Arm
import org.team9432.frc2025.robot.subsystems.superstructure.dispenser.Dispenser
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

// Inspired by 6328 <3
// https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/244#p-3503708-implementation-part-one-structure-4
class Superstructure(private val elevator: Elevator, private val arm: Arm, private val dispenser: Dispenser) :
    SubsystemBase() {
    enum class State {
        STOW,
        TEST_ARM,
        INTAKE_CORAL,
        PREPARE_TALL_SCORE,
        PREPARE_L2,
        PREPARE_L3,
        PREPARE_L4,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
    }

    private val transitions = TransitionCommands(elevator, arm, dispenser)
    private val visualizer = SuperstructureVisualizer("Superstructure/Poses")

    /** The latest complete state of the system. */
    private var currentState: State = STOW

    /** The current state being moved towards on a path to [goal]. */
    private var step: State? = null

    /** The current targeted state of the system. */
    private var goal: State = STOW

    /** The current command running between states. */
    private var currentMovementCommand: Command = Commands.none()

    private var stateTrackingDisabled = false

    override fun periodic() {
        dispenser.periodic()

        if (!stateTrackingDisabled) {
            trackToNextState()
        }

        visualizer.publish(elevatorMeters = elevator.positionMeters, armRotations = arm.positionRotations)

        Logger.recordOutput("Superstructure/StateTrackingEnabled", !stateTrackingDisabled)
        Logger.recordOutput("Superstructure/Homed", elevator.hasHomed && arm.hasHomed)
        Logger.recordOutput("Superstructure/CurrentState", currentState)
        Logger.recordOutput("Superstructure/StepState", step)
        Logger.recordOutput("Superstructure/GoalState", goal)
    }

    fun runToGoal(goal: State) = runOnce { updateGoal(goal) }.withName("Superstructure Goal $goal")

    private fun trackToNextState() {
        // If there isn't a command running, but we still have a step state set, the move to that
        // step was just completed
        if (!currentMovementCommand.isScheduled && step != null) {
            // Update our current state
            currentState = step!!
            step = null
        }

        // If the robot is not at the goal
        if (currentState != goal) {
            // Find the next state and the command to move to it
            val nextState = getStepBetween(start = currentState, goal = goal)
            step = nextState
            currentMovementCommand = transitions[currentState to nextState]
            currentMovementCommand.schedule()
        }
    }

    private fun updateGoal(newGoal: State) {
        // Don't bother if it's already the target
        if (newGoal == goal) return

        // Update the goal state
        goal = newGoal

        // Don't do anything if tracking is disabled
        if (stateTrackingDisabled) return

        // Return if not already moving, if moving its more complicated
        if (step == null) return

        // Get the ideal next state from where we are going
        val idealStep = getStepBetween(step!!, newGoal)

        // If it's the same position, nothing to change
        if (idealStep == step) return

        // If it's back where we were, it's safe to cancel and reverse
        if (idealStep == currentState) {
            // Otherwise, set our current state to where we were going and move backwards
            currentMovementCommand.cancel()
            currentMovementCommand = transitions[step to currentState]
            currentMovementCommand.schedule()
            val temp = currentState
            currentState = step!!
            step = temp
        }
    }

    fun getStepBetween(start: State, goal: State): State? {
        val visited = mutableMapOf<State, State?>()
        val queue = ArrayDeque<State>()
        queue.add(start)
        visited[start] = null
        while (queue.isNotEmpty()) {
            val current = queue.removeFirst()
            if (current == goal) break

            val neighbors = transitions.allValidMoves.filter { it.first == current }.map { it.second }

            for (neighbor in neighbors) {
                if (neighbor !in visited.keys) {
                    queue.add(neighbor)
                    visited[neighbor] = current
                }
            }
        }

        if (!visited.containsKey(goal)) {
            DriverStation.reportError("FAILED TO FIND PATH BETWEEN $start AND $goal", false)
            return null
        }

        // Trace back the path from goal to start
        var nextState: State = goal
        while (nextState != start) {
            val parent = visited[nextState]

            if (parent == null) {
                // No valid path found
                return null
            } else if (parent == start) {
                // Return the edge from start to the next node
                return nextState
            }
            nextState = parent
        }
        return nextState
    }

    fun homeSystem(): Command =
        Commands.sequence(
                elevator.homeElevator(),
                elevator.runToGoal(Elevator.Goal.MIN_ARM_OUT),
                arm.homeArm(),
                elevator.runToGoal(Elevator.Goal.STOW),
                arm.runToGoal(Arm.Goal.STOW),
            )
            .beforeStarting({ stateTrackingDisabled = true })
            .finallyDo { _ -> stateTrackingDisabled = false }
            .chainAddRequirements(this)
            .withName("Home Superstructure")

    fun elevatorStaticCharacterization() =
        elevator
            .staticCharacterization()
            .beforeStarting({ stateTrackingDisabled = true })
            .finallyDo { _ -> stateTrackingDisabled = false }
            .chainAddRequirements(this)
            .withName("Elevator Static Characterization")

    fun armStaticCharacterization() =
        arm.staticCharacterization()
            .beforeStarting({ stateTrackingDisabled = true })
            .finallyDo { _ -> stateTrackingDisabled = false }
            .chainAddRequirements(this)
            .withName("Arm Static Characterization")
}
