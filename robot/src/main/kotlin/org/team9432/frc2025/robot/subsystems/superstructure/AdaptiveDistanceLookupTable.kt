package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import org.team9432.frc2025.robot.subsystems.superstructure.arm.Arm
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

object AdaptiveDistanceLookupTable {
    val L2 =
        LookupTable().apply {
            addMapValue(0.0, Elevator.Goal.SCORE_L2.meters, Arm.Goal.SCORE_L2.rotations)
            addMapValue(0.1, Elevator.Goal.SCORE_L2.meters + Units.inchesToMeters(2.0), Arm.Goal.SCORE_L2.rotations)
        }
    val L3 =
        LookupTable().apply {
            addMapValue(0.0, Elevator.Goal.SCORE_L3.meters, Arm.Goal.SCORE_L3.rotations)
            addMapValue(0.1, Elevator.Goal.SCORE_L3.meters + Units.inchesToMeters(2.0), Arm.Goal.SCORE_L3.rotations)
        }

    class LookupTable {
        private val elevatorHeightMap = InterpolatingDoubleTreeMap()
        private val armAngleMap = InterpolatingDoubleTreeMap()

        fun addMapValue(distance: Double, elevatorHeight: Double, armAngle: Double) {
            elevatorHeightMap.put(distance, elevatorHeight)
            armAngleMap.put(distance, armAngle)
        }

        fun getElevatorHeight(distance: Double) = elevatorHeightMap.get(distance)

        fun getArmAngle(distance: Double) = armAngleMap.get(distance)
    }
}
