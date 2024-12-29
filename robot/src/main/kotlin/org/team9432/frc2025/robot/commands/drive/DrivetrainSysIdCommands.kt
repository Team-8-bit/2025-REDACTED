package org.team9432.frc2025.robot.commands.drive

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.subsystems.drive.Drive
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants

class DrivetrainSysIdCommands(private val drive: Drive) {
    // Config holds information about how the test will be run
    private val config = SysIdRoutine.Config(
        /* rampRate = */ Volts.per(Second).of(1.0),
        /* stepVoltage = */ Volts.of(7.0),
        /* timeout = */ Seconds.of(10.0),
        /* recordState = */ { state -> Logger.recordOutput("Drive/SysIdState", state.toString()) }
    )

    // Mechanism to run the drivetrain in a linear direction
    private val linearModuleDirections = Array(4) { Rotation2d.kZero }
    private val linearMechanism = SysIdRoutine.Mechanism(
        /* drive = */ { voltage -> drive.runDriveCharacterizationVoltage(voltage.`in`(Volts), linearModuleDirections) },
        /* log = */ null,
        /* subsystem = */ drive
    )

    // Mechanism to spin the drivetrain in place
    private val angularModuleDirections = DrivetrainConstants.MODULE_TRANSLATIONS.map { Rotation2d(it.x, it.y).plus(Rotation2d.kCCW_Pi_2) }.toTypedArray()
    private val angularMechanism = SysIdRoutine.Mechanism(
        /* drive = */ { voltage -> drive.runDriveCharacterizationVoltage(voltage.`in`(Volts), angularModuleDirections) },
        /* log = */ null,
        /* subsystem = */ drive
    )

    // The two different routines we want to test
    private val linearRoutine = SysIdRoutine(config, linearMechanism)
    private val angularRoutine = SysIdRoutine(config, angularMechanism)

    /**
     * A command to reset the drive modules to the target steer setpoint. Used before
     * running a characterization routine to ensure the modules are already in the
     * right position and won't affect the data by rotating after it starts.
     */
    private fun resetModules(steerSetpoints: Array<Rotation2d>): Command {
        return drive.run { drive.runDriveCharacterizationVoltage(0.0, steerSetpoints) }.withTimeout(1.0)
    }

    val linearQuasistaticForward: Command = resetModules(linearModuleDirections).andThen(linearRoutine.quasistatic(SysIdRoutine.Direction.kForward))
    val linearQuasistaticReverse: Command = resetModules(linearModuleDirections).andThen(linearRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
    val linearDynamicForward: Command = resetModules(linearModuleDirections).andThen(linearRoutine.dynamic(SysIdRoutine.Direction.kForward))
    val linearDynamicReverse: Command = resetModules(linearModuleDirections).andThen(linearRoutine.dynamic(SysIdRoutine.Direction.kReverse))
    val angularQuasistaticForward: Command = resetModules(angularModuleDirections).andThen(angularRoutine.quasistatic(SysIdRoutine.Direction.kForward))
    val angularQuasistaticReverse: Command = resetModules(angularModuleDirections).andThen(angularRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
    val angularDynamicForward: Command = resetModules(angularModuleDirections).andThen(angularRoutine.dynamic(SysIdRoutine.Direction.kForward))
    val angularDynamicReverse: Command = resetModules(angularModuleDirections).andThen(angularRoutine.dynamic(SysIdRoutine.Direction.kReverse))
}