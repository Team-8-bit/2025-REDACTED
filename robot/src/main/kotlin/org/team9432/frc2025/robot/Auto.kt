package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.team9432.frc2025.lib.util.applyFlip
import org.team9432.frc2025.lib.util.not
import org.team9432.frc2025.robot.FieldConstants.CoralStation
import org.team9432.frc2025.robot.FieldConstants.Reef.Branch
import org.team9432.frc2025.robot.ScoringState.CoralScoringTarget
import org.team9432.frc2025.robot.commands.drive.DriveToPose
import org.team9432.frc2025.robot.subsystems.drive.Drive
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.subsystems.rollers.Rollers
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure

class Auto(
    private val robotPosition: RobotPosition,
    private val localizer: Localizer,
    private val drive: Drive,
    private val superstructure: Superstructure,
    private val rollers: Rollers,
    private val scoringState: ScoringState,
) {
    fun initializeAuto(): Command = superstructure.fakeAutoHome().alongWith(rollers.preloadCoral().asProxy())

    private val coralStationTransform = Transform2d(DrivetrainConstants.BUMPER_LENGTH / 2, -1.0, Rotation2d.kZero)

    val autoAlignForStationPickup =
        DriveToPose(
            drive,
            localizer,
            { scoringState.autoCoralStationPose?.applyFlip() ?: localizer.estimatedPose },
            { localizer.estimatedPose },
        )

    fun onlyL2(branch: Branch): Command =
        Commands.defer({ initializeAuto().andThen(preloadAndScore(branch, CoralScoringTarget.L2)) }, emptySet())

    fun auto(moves: List<Pair<Branch, CoralScoringTarget>>, coralStation: CoralStation): Command =
        Commands.defer(
            {
                if (moves.isEmpty()) return@defer Commands.none()

                val commands =
                    Array(moves.size) { index ->
                        val (branch, level) = moves[index]
                        if (index == 0) {
                            preloadAndScore(branch, level)
                        } else {
                            pickupAndScore(branch, level, coralStation)
                        }
                    }

                initializeAuto().andThen(*commands)
            },
            emptySet(),
        )

    fun maxL4Left(): Command =
        Commands.defer(
            {
                auto(
                    listOf(
                        Pair(Branch.J, CoralScoringTarget.L4),
                        Pair(Branch.K, CoralScoringTarget.L4),
                        Pair(Branch.L, CoralScoringTarget.L4),
                        Pair(Branch.A, CoralScoringTarget.L4),
                    ),
                    CoralStation.LEFT,
                )
            },
            emptySet(),
        )

    private fun preloadAndScore(branch: Branch, level: CoralScoringTarget) =
        Commands.sequence(
            Commands.runOnce({
                scoringState.autoCoralTarget = level
                scoringState.autoBranchTarget = branch
            }),
            Commands.waitUntil((!rollers.hasCoralTrigger)),
            Commands.waitSeconds(0.25),
        )

    private fun pickupAndScore(branch: Branch, level: CoralScoringTarget, coralStation: CoralStation) =
        Commands.sequence(
            Commands.runOnce({
                scoringState.autoCoralTarget = level
                scoringState.autoBranchTarget = branch
                scoringState.autoCoralStationPose = coralStation.centerPose.transformBy(coralStationTransform)
            }),
            Commands.waitUntil(rollers.hasCoralTrigger.or(autoAlignForStationPickup::atGoal)),
            Commands.runOnce({ scoringState.autoCoralStationPose = null }),
            Commands.waitUntil(rollers.hasCoralTrigger).withTimeout(2.0),
            Commands.waitUntil((!rollers.hasCoralTrigger)),
            Commands.waitSeconds(0.25),
        )
}
