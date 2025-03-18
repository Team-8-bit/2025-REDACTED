package org.team9432.frc2025.robot.subsystems.superstructure

import org.team9432.frc2025.robot.subsystems.superstructure.arm.Arm
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

enum class SuperstructureState(val elevatorGoal: Elevator.Goal, val armGoal: Arm.Goal) {
    STOW(elevatorGoal = Elevator.Goal.STOW, armGoal = Arm.Goal.STOW),
    ARM_ABOVE_BUMPER(elevatorGoal = Elevator.Goal.MIN_ARM_OUT, armGoal = Arm.Goal.STOW),
    PREPARE_L1(elevatorGoal = Elevator.Goal.L1, armGoal = Arm.Goal.L1),
    PREPARE_L2(elevatorGoal = Elevator.Goal.L2, armGoal = Arm.Goal.L2),
    PREPARE_L3(elevatorGoal = Elevator.Goal.L3, armGoal = Arm.Goal.L3),
    PREPARE_L4(elevatorGoal = Elevator.Goal.L4, armGoal = Arm.Goal.L4),
    L4_PREP(elevatorGoal = Elevator.Goal.L4_PREP, armGoal = Arm.Goal.L4),
    ALGAE_STOW(elevatorGoal = Elevator.Goal.HOLD_ALGAE_LOW, armGoal = Arm.Goal.HOLD_ALGAE_LOW),
    INTAKE_ALGAE_LOW(elevatorGoal = Elevator.Goal.INTAKE_ALGAE_REEF_LOW, armGoal = Arm.Goal.INTAKE_ALGAE_REEF),
    INTAKE_ALGAE_HIGH(elevatorGoal = Elevator.Goal.INTAKE_ALGAE_REEF_HIGH, armGoal = Arm.Goal.INTAKE_ALGAE_REEF),
    PREPARE_NET(elevatorGoal = Elevator.Goal.PREPARE_NET, armGoal = Arm.Goal.PREPARE_NET),
    SCORE_NET(elevatorGoal = Elevator.Goal.SCORE_NET, armGoal = Arm.Goal.SCORE_NET),
    ALGAE_FLOOR(elevatorGoal = Elevator.Goal.STOW, armGoal = Arm.Goal.FLOOR_ALGAE),
    UNJAM_CORAL(elevatorGoal = Elevator.Goal.UNJAM_CORAL, armGoal = Arm.Goal.UNJAM_CORAL),
    PROCESSOR(elevatorGoal = Elevator.Goal.PREPARE_PROCESSOR, armGoal = Arm.Goal.PREPARE_PROCESSOR);

    val isCoralScoring
        get() = this in setOf(PREPARE_L1, PREPARE_L2, PREPARE_L3, PREPARE_L4)

    val isAlgaeScoring
        get() = this in setOf(PREPARE_NET, SCORE_NET, PROCESSOR)
}
