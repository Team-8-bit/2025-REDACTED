package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.team9432.frc2025.lib.util.not
import org.team9432.frc2025.robot.RobotState.CoralScoringTarget
import org.team9432.frc2025.robot.commands.drive.DriveToPose
import org.team9432.frc2025.robot.subsystems.drive.Drive
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.subsystems.rollers.Rollers
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure
import org.team9432.frc2025.robot.util.FieldConstants.CoralStation
import org.team9432.frc2025.robot.util.FieldConstants.Reef.Branch
import org.team9432.frc2025.robot.util.FieldConstants.Reef.StagedAlgae

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
                { robotState.autoCoralStationPose ?: localizer.estimatedPose },
                { localizer.estimatedPose },
            )
            .apply { name = "AutoAlignForStationPickup" }

    fun onlyL2(branch: Branch): Command =
        Commands.defer({ initializeAuto().andThen(preloadAndScore(branch, CoralScoringTarget.L2)) }, emptySet())

    fun onlyL4(branch: Branch): Command =
        Commands.defer({ initializeAuto().andThen(preloadAndScore(branch, CoralScoringTarget.L4)) }, emptySet())

    fun coralAuto(moves: List<Pair<Branch, CoralScoringTarget>>, coralStationPose: Pose2d): Command =
        Commands.defer(
            {
                if (moves.isEmpty()) return@defer Commands.none()

                val moveQueue = moves.toMutableList()

                val preloadMove = moveQueue.removeFirst()
                val preload = preloadAndScore(preloadMove.first, preloadMove.second)

                val nextCommands =
                    Commands.defer(
                            {
                                if (moveQueue.isEmpty()) {
                                    // This probably can't happen
                                    Commands.none()
                                } else {
                                    val (branch, level) = moveQueue.first()
                                    pickupAndScore(branch, level, coralStationPose) {
                                        // Successful coral pickup, this branch will be scored on
                                        moveQueue.removeFirst()
                                    }
                                }
                            },
                            emptySet(),
                        )
                        .repeatedly()
                        .until(moveQueue::isEmpty)

                initializeAuto().andThen(preload).andThen(nextCommands)
            },
            emptySet(),
        )

    fun maxL4Left(): Command =
        Commands.defer(
            {
                coralAuto(
                    listOf(
                        Pair(Branch.J, CoralScoringTarget.L4),
                        Pair(Branch.K, CoralScoringTarget.L4),
                        Pair(Branch.L, CoralScoringTarget.L4),
                        Pair(Branch.A, CoralScoringTarget.L4),
                    ),
                    CoralStation.ALLIANCE_LEFT,
                )
            },
            emptySet(),
        )

    fun maxL4Right(): Command =
        Commands.defer(
            {
                coralAuto(
                    listOf(
                        Pair(Branch.E, CoralScoringTarget.L4),
                        Pair(Branch.D, CoralScoringTarget.L4),
                        Pair(Branch.C, CoralScoringTarget.L4),
                        Pair(Branch.B, CoralScoringTarget.L4),
                    ),
                    CoralStation.ALLIANCE_RIGHT,
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

    private fun pickupAndScore(
        branch: Branch,
        level: CoralScoringTarget,
        coralStationPose: Pose2d,
        onCoralPickup: () -> Unit = {},
    ) =
        Commands.sequence(
            Commands.runOnce({
                robotState.autoCoralTarget = level
                robotState.autoBranchTarget = branch
                robotState.autoCoralStationPose = coralStationPose.transformBy(coralStationTransform)
            }),
            Commands.waitUntil(
                rollers.hasCoralTrigger.or {
                    autoAlignForStationPickup.withinTolerance(2.0, Units.degreesToRotations(5.0))
                }
            ),
            // Commands.waitSeconds(0.5), // This works, add if needed to pause at the coral station
            Commands.runOnce({ robotState.autoCoralStationPose = null }),
            Commands.waitUntil(rollers.hasCoralTrigger)
                .finallyDo { interrupted -> if (!interrupted) onCoralPickup.invoke() }
                .withTimeout(2.0),
            Commands.waitUntil(!rollers.hasCoralTrigger),
            Commands.waitSeconds(0.3),
        )

    fun algaeAuto(vararg algae: StagedAlgae) =
        Commands.defer(
            {
                initializeAuto()
                    .andThen(preloadAndScore(Branch.H, CoralScoringTarget.L4))
                    .andThen(*algae.map { pickupAndNetAlgae(it) }.toTypedArray())
            },
            emptySet(),
        )

    private fun pickupAndNetAlgae(position: StagedAlgae) =
        Commands.sequence(
            Commands.runOnce({ robotState.autoAlgaePickupTarget = position }),
            Commands.waitUntil(rollers.hasAlgaeTrigger.and(robotPosition.isSafeToUseArm)),
            Commands.runOnce({
                robotState.autoAlgaePickupTarget = null
            }), // Algae autoalign will drive back, this lets it start the net score
            Commands.waitUntil(!rollers.hasAlgaeTrigger),
            Commands.waitSeconds(0.4),
        )
}
