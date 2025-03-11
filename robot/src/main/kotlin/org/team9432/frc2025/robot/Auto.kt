package org.team9432.frc2025.robot

import edu.wpi.first.wpilibj2.command.Commands
import org.team9432.frc2025.robot.commands.drive.DriveToPose
import org.team9432.frc2025.robot.subsystems.drive.Drive
import org.team9432.frc2025.robot.subsystems.rollers.Rollers
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure

class Auto(
    private val robotPosition: RobotPosition,
    private val localizer: Localizer,
    private val drive: Drive,
    private val superstructure: Superstructure,
    private val rollers: Rollers,
) {
    fun initializeAuto() = superstructure.fakeAutoHome().alongWith(rollers.preloadCoral())

    fun simpleL2() =
        Commands.parallel(
            initializeAuto(),
            DriveToPose(drive, localizer, { robotPosition.getActiveBranchAlignPose(FieldConstants.Reef.Branch.B) }),
        )
}
