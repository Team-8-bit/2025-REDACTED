package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.collections.set
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure.State.*
import org.team9432.frc2025.robot.subsystems.superstructure.coralarm.CoralArm
import org.team9432.frc2025.robot.subsystems.superstructure.dispenser.Dispenser
import org.team9432.frc2025.robot.subsystems.superstructure.coralarm.CoralArmConstants
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

// Inspired by 6328 <3:
// https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/244#p-3503708-implementation-part-one-structure-4
class Superstructure(private val elevator: Elevator, private val coralArm: CoralArm, private val dispenser: Dispenser) :
    SubsystemBase() {
    private val transitionCommands: Map<Pair<State, State>, Command>

    private var currentState: State = STOW
    private var stepState: State? = null
    private var goalState: State = STOW

    private var currentMovemnetCommand: Command = Commands.none()

    init {
        defaultCommand = runGoal(STOW)
    }

    override fun periodic() {
        if (!currentMovemnetCommand.isScheduled) {
            // If there isn't a command running but we still have a step state set, the move to that
            // step was just completed
            if (stepState != null) {
                // Update our current state
                currentState = stepState!!
                stepState = null
            }
        }

        // If we aren't yet at the goal
        if (currentState != goalState) {
            // Find the next state and the command to move to it
            val nextState = getCommandBetween(start = currentState, goal = goalState)
            currentMovemnetCommand =
                transitionCommands[currentState to nextState]
                    ?: Commands.print("Failed to fetch command between $currentState and $goalState.")
            currentMovemnetCommand.schedule()
        }

        elevator.periodic()
        coralArm.periodic()

        Logger.recordOutput(
            "Superstructure/Poses/A_Stage2",
            Pose3d(0.0, 0.0, elevator.positionMeters, Rotation3d.kZero),
        )
        Logger.recordOutput(
            "Superstructure/Poses/B_CoralArm",
            Pose3d(
                Units.inchesToMeters(-8.25),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(19.157754 + elevator.positionMeters),
                Rotation3d(
                    0.0,
                    Units.rotationsToRadians(coralArm.positionRotations - CoralArmConstants.MIN_POSITION),
                    0.0,
                ),
            ),
        )

        Logger.recordOutput("Superstructure/CurrentState", currentState)
        Logger.recordOutput("Superstructure/StepState", stepState)
        Logger.recordOutput("Superstructure/GoalState", goalState)
    }

    fun runGoal(goal: State) = runOnce { setGoal(goal) }.until { goalState == goal }

    private fun setGoal(newGoal: State) {
        // Don't bother if it's already the target
        if (newGoal == goalState) return

        // Update the goal state
        goalState = newGoal
    }

    enum class State {
        STOW,
        INTAKE_CORAL,
        PREPARE_TALL_SCORE,
        PREPARE_L2,
        PREPARE_L3,
        PREPARE_L4,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
    }

    private fun runElevatorToGoal(goal: Elevator.Goal) =
        Commands.sequence(runOnce { elevator.goal = goal }, Commands.waitUntil(elevator::atGoal))

    private fun runArmToGoal(goal: CoralArm.Goal) =
        Commands.sequence(runOnce { coralArm.goal = goal }, Commands.waitUntil(coralArm::atGoal))

    private fun runDispenser(goal: Dispenser.Goal) = runOnce { dispenser.goal = goal }

    // Builds the set of legal moves
    init {
        val transitions = mutableMapOf<Pair<State, State>, Command>()

        transitions[STOW to PREPARE_TALL_SCORE] =
            Commands.sequence(runElevatorToGoal(Elevator.Goal.MIN_ARM_OUT), runArmToGoal(CoralArm.Goal.PREPARE_SCORE))

        transitions[PREPARE_TALL_SCORE to STOW] =
            Commands.sequence(runArmToGoal(CoralArm.Goal.STOW), runElevatorToGoal(Elevator.Goal.STOW))

        transitions[STOW to INTAKE_CORAL] = runDispenser(Dispenser.Goal.INTAKE)
        transitions[INTAKE_CORAL to STOW] = runDispenser(Dispenser.Goal.IDLE)

        transitions[PREPARE_TALL_SCORE to PREPARE_L2] = runElevatorToGoal(Elevator.Goal.L2)
        transitions[PREPARE_TALL_SCORE to PREPARE_L3] = runElevatorToGoal(Elevator.Goal.L3)
        transitions[PREPARE_TALL_SCORE to PREPARE_L4] = runElevatorToGoal(Elevator.Goal.L4)

        for (scoringGoal in setOf(PREPARE_L2, PREPARE_L3, PREPARE_L4)) {
            transitions[scoringGoal to PREPARE_TALL_SCORE] = runElevatorToGoal(Elevator.Goal.MIN_ARM_OUT)
        }

        transitions[PREPARE_L2 to SCORE_L2] = runDispenser(Dispenser.Goal.OUTTAKE)
        transitions[SCORE_L2 to PREPARE_L2] = runDispenser(Dispenser.Goal.IDLE)
        transitions[PREPARE_L3 to SCORE_L3] = runDispenser(Dispenser.Goal.OUTTAKE)
        transitions[SCORE_L3 to PREPARE_L3] = runDispenser(Dispenser.Goal.IDLE)
        transitions[PREPARE_L4 to SCORE_L4] = runDispenser(Dispenser.Goal.OUTTAKE)
        transitions[SCORE_L4 to PREPARE_L4] = runDispenser(Dispenser.Goal.IDLE)

        transitionCommands = transitions
    }

    fun getCommandBetween(start: State, goal: State): State? {
        val visited = mutableMapOf<State, State?>()
        val queue = ArrayDeque<State>()
        queue.add(start)
        visited[start] = null
        while (queue.isNotEmpty()) {
            val current = queue.removeFirst()
            if (current == goal) break

            val neighbors = transitionCommands.keys.filter { it.first == current }.map { it.second }

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

    fun runElevatorCharacterizationAmps(amps: Double) {
        elevator.characterizationInput = amps
    }

    fun getElevatorCharacterizationVelocity(): Double {
        return elevator.velocityMps
    }

    fun endElevatorCharacterization() {
        elevator.characterizationInput = null
    }
}
