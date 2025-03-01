package org.team9432.frc2025.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.sequence
import org.team9432.frc2025.robot.ScoringState
import org.team9432.frc2025.robot.subsystems.rollers.Rollers
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure

class IntakeGamePiece(
    private val superstructure: Superstructure,
    private val rollers: Rollers,
    private val scoringState: ScoringState,
) {
    fun command(): Command {
        return sequence(
            superstructure.runToGoal(Superstructure.State.INTAKE_CORAL),
            rollers.runGoal(Rollers.State.INTAKE_CORAL),
        )
    }
}
