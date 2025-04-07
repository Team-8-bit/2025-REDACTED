package org.team9432.frc2025.robot.subsystems.superstructure

import org.team9432.frc2025.robot.subsystems.superstructure.arm.Arm
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

enum class SuperstructureState(val elevatorGoal: Elevator.Goal, val armGoal: Arm.Goal) {
    STOW(elevatorGoal = Elevator.Goal.STOW, armGoal = Arm.Goal.STOW),
    ARM_ABOVE_BUMPER(elevatorGoal = Elevator.Goal.MIN_ARM_OUT, armGoal = Arm.Goal.STOW),
    SCORE_L1(elevatorGoal = Elevator.Goal.SCORE_L1, armGoal = Arm.Goal.SCORE_L1),
    SCORE_L2(elevatorGoal = Elevator.Goal.SCORE_L2, armGoal = Arm.Goal.SCORE_L2),
    SCORE_L3(elevatorGoal = Elevator.Goal.SCORE_L3, armGoal = Arm.Goal.SCORE_L3),
    PLACE_L4(elevatorGoal = Elevator.Goal.PLACE_L4, armGoal = Arm.Goal.PLACE_L4),
    SCORE_L4(elevatorGoal = Elevator.Goal.SCORE_L4, armGoal = Arm.Goal.SCORE_L4),
    PREP_L4(elevatorGoal = Elevator.Goal.PREP_L4, armGoal = Arm.Goal.SCORE_L4),
    ALGAE_STOW(elevatorGoal = Elevator.Goal.HOLD_ALGAE_LOW, armGoal = Arm.Goal.HOLD_ALGAE_LOW),
    INTAKE_ALGAE_LOW(elevatorGoal = Elevator.Goal.INTAKE_ALGAE_REEF_LOW, armGoal = Arm.Goal.INTAKE_ALGAE_REEF),
    INTAKE_ALGAE_HIGH(elevatorGoal = Elevator.Goal.INTAKE_ALGAE_REEF_HIGH, armGoal = Arm.Goal.INTAKE_ALGAE_REEF),
    PREP_NET(elevatorGoal = Elevator.Goal.PREP_NET, armGoal = Arm.Goal.PREP_NET),
    SCORE_NET(elevatorGoal = Elevator.Goal.SCORE_NET, armGoal = Arm.Goal.SCORE_NET),
    ALGAE_FLOOR(elevatorGoal = Elevator.Goal.STOW, armGoal = Arm.Goal.FLOOR_ALGAE),
    UNJAM_CORAL(elevatorGoal = Elevator.Goal.UNJAM_CORAL, armGoal = Arm.Goal.UNJAM_CORAL),
    PROCESSOR(elevatorGoal = Elevator.Goal.PREPARE_PROCESSOR, armGoal = Arm.Goal.PREPARE_PROCESSOR);

    val isCoralScoring
        get() = this in setOf(SCORE_L1, SCORE_L2, SCORE_L3, PLACE_L4, SCORE_L4, PREP_L4)

    val isAlgaeScoring
        get() = this in setOf(PREP_NET, SCORE_NET, PROCESSOR)
}
