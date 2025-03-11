package org.team9432.frc2025.robot.subsystems.superstructure

import kotlin.test.Test
import kotlin.test.assertNotNull
import org.team9432.frc2025.robot.subsystems.superstructure.arm.Arm
import org.team9432.frc2025.robot.subsystems.superstructure.arm.ArmIO
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.ElevatorIO

internal class SuperstructureSequencerTest {
    @Test
    fun allStatesReachable() {
        val superstructure = Superstructure(Elevator(object : ElevatorIO {}), Arm(object : ArmIO {}))

        // Build a list of all possible movements
        val allPossibleMovements = mutableListOf<Pair<SuperstructureState, SuperstructureState>>()

        val currentList = SuperstructureState.entries.toMutableList()
        while (currentList.isNotEmpty()) {
            val firstItem = currentList.first()
            currentList.removeFirst()

            for (otherItem in currentList) {
                allPossibleMovements.add(firstItem to otherItem)
                allPossibleMovements.add(otherItem to firstItem)
            }
        }

        for (movement in allPossibleMovements) {
            val initialState = movement.first
            val targetState = movement.second
            var currentState = initialState

            // Commented out stuff is just to print the list of steps from each state to each other
            // state which I thought was cool
            //            val path =
            // mutableListOf<org.team9432.frc2025.robot.subsystems.superstructure.State>()
            //            path.add(initialState)

            while (currentState != targetState) {
                val nextState = superstructure.getStepBetween(currentState, targetState)

                assertNotNull(nextState, "Failed to find a path between $initialState and ${targetState}!")

                //                path.add(nextState)
                currentState = nextState
            }

            //            println("Path between $initialState and $targetState: $path")
        }
    }
}
