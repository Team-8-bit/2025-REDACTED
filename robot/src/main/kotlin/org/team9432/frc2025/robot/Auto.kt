package org.team9432.frc2025.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.team9432.frc2025.lib.util.not
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
    fun initializeAuto() = superstructure.fakeAutoHome().alongWith(rollers.preloadCoral().asProxy())

    private fun setNextScoreGoal(branch: FieldConstants.Reef.Branch, height: ScoringState.CoralScoringTarget): Command =
        Commands.runOnce({
            scoringState.autoCoralTarget = height
            scoringState.autoBranchTarget = branch
        })

    private fun driveToPose(targetPose: () -> Pose2d, robotPose: () -> Pose2d) =
        DriveToPose(drive, localizer, targetPose, robotPose)

    private val coralStationTransform = Transform2d(-DrivetrainConstants.BUMPER_LENGTH / 2, 0.0, Rotation2d.kZero)

    fun simpleL2(branch: FieldConstants.Reef.Branch): Command =
        Commands.defer(
            {
                initializeAuto()
                    .andThen(
                        Commands.runOnce({
                            scoringState.autoCoralTarget = ScoringState.CoralScoringTarget.L2
                            scoringState.autoBranchTarget = branch
                        })
                    )
            },
            emptySet(),
        )

    fun simpleL2L4(l2Branch: FieldConstants.Reef.Branch, l4Branch: FieldConstants.Reef.Branch): Command =
        Commands.defer(
            {
                initializeAuto()
                    .andThen(
                        setNextScoreGoal(l2Branch, ScoringState.CoralScoringTarget.L2),
                        Commands.waitUntil(!rollers.hasCoralTrigger.debounce(.5)),
                        Commands.runOnce({
                            scoringState.autoCoralTarget = ScoringState.CoralScoringTarget.L4
                            scoringState.autoBranchTarget = l4Branch
                            scoringState.autoCoralStationPose = FieldConstants.CoralStation.leftCenterFace.transformBy(coralStationTransform)
                        }),
                    )
            },
            emptySet(),
        )
}
