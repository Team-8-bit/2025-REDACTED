package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.team9432.frc2025.lib.util.applyFlip
import org.team9432.frc2025.lib.util.not
import org.team9432.frc2025.robot.RobotState.CoralScoringTarget
import org.team9432.frc2025.robot.commands.drive.DriveToPose
import org.team9432.frc2025.robot.subsystems.drive.Drive
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.subsystems.rollers.Rollers
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure
import org.team9432.frc2025.robot.util.FieldConstants.CoralStation
import org.team9432.frc2025.robot.util.FieldConstants.Reef.Branch

class Auto(
    private val robotPosition: RobotPosition,
    private val localizer: Localizer,
    private val drive: Drive,
    private val superstructure: Superstructure,
    private val rollers: Rollers,
    private val robotState: RobotState,
) {
    fun initializeAuto(): Command = superstructure.fakeAutoHome().alongWith(rollers.preloadCoral().asProxy())

    private val coralStationTransform = Transform2d((DrivetrainConstants.BUMPER_LENGTH / 2), 0.0, Rotation2d.kZero)

    val autoAlignForStationPickup =
        DriveToPose(
                drive,
                localizer,
                { robotState.autoCoralStationPose?.applyFlip() ?: localizer.estimatedPose },
                { localizer.estimatedPose },
            )
            .apply { name = "AutoAlignForStationPickup" }

    fun onlyL2(branch: Branch): Command =
        Commands.defer({ initializeAuto().andThen(preloadAndScore(branch, CoralScoringTarget.L2)) }, emptySet())

    fun onlyL4(branch: Branch): Command =
        Commands.defer({ initializeAuto().andThen(preloadAndScore(branch, CoralScoringTarget.L4)) }, emptySet())

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

    fun maxL4Right(): Command =
        Commands.defer(
            {
                auto(
                    listOf(
                        Pair(Branch.E, CoralScoringTarget.L4),
                        Pair(Branch.D, CoralScoringTarget.L4),
                        Pair(Branch.C, CoralScoringTarget.L4),
                        Pair(Branch.B, CoralScoringTarget.L4),
                    ),
                    CoralStation.RIGHT,
                )
            },
            emptySet(),
        )

    fun maxL4LeftNoFront(): Command =
        Commands.defer(
            {
                auto(
                    listOf(
                        Pair(Branch.J, CoralScoringTarget.L4),
                        Pair(Branch.K, CoralScoringTarget.L4),
                        Pair(Branch.L, CoralScoringTarget.L4),
                        Pair(Branch.L, CoralScoringTarget.L4),
                    ),
                    CoralStation.LEFT,
                )
            },
            emptySet(),
        )

    fun maxL4RightNoFront(): Command =
        Commands.defer(
            {
                auto(
                    listOf(
                        Pair(Branch.E, CoralScoringTarget.L4),
                        Pair(Branch.D, CoralScoringTarget.L4),
                        Pair(Branch.C, CoralScoringTarget.L4),
                        Pair(Branch.C, CoralScoringTarget.L4),
                    ),
                    CoralStation.RIGHT,
                )
            },
            emptySet(),
        )

    private fun preloadAndScore(branch: Branch, level: CoralScoringTarget) =
        Commands.sequence(
            Commands.runOnce({
                robotState.autoCoralTarget = level
                robotState.autoBranchTarget = branch
            }),
            Commands.waitUntil(!rollers.hasCoralTrigger),
            Commands.waitSeconds(0.3),
        )

    private fun pickupAndScore(branch: Branch, level: CoralScoringTarget, coralStation: CoralStation) =
        Commands.sequence(
            Commands.runOnce({
                robotState.autoCoralTarget = level
                robotState.autoBranchTarget = branch
                robotState.autoCoralStationPose = coralStation.centerPose.transformBy(coralStationTransform)
            }),
            Commands.waitUntil(
                rollers.hasCoralTrigger.or {
                    autoAlignForStationPickup.withinTolerance(2.0, Units.degreesToRotations(5.0))
                }
            ),
            // Commands.waitSeconds(0.5), // This works, add if needed to pause at the coral station
            Commands.runOnce({ robotState.autoCoralStationPose = null }),
            Commands.waitUntil(rollers.hasCoralTrigger).withTimeout(2.5),
            Commands.waitUntil(!rollers.hasCoralTrigger),
            Commands.waitSeconds(0.3),
        )
}
