// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package org.team9432.frc2025.robot.commands.drive

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.abs
import kotlin.math.min
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.subsystems.drive.Drive

// By 6328:
// https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/63e5db66eee847360567ef24c3d5807280b300e1/src/main/java/org/littletonrobotics/frc2025/commands/DriveToPose.java
class DriveToPose(
    private val drive: Drive,
    private val localizer: Localizer,
    private val targetPose: () -> Pose2d,
    private val robotPose: () -> Pose2d = { localizer.estimatedPose },
) : Command() {
    private val driveController = ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(0.0, 0.0))
    private val thetaController = ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(0.0, 0.0))

    private var lastSetpointTranslation = Translation2d()
    private var driveErrorAbs = 0.0
    private var thetaErrorAbs = 0.0

    private var running = false

    init {
        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-0.5, 0.5)

        addRequirements(drive)
    }

    override fun initialize() {
        val currentPose = robotPose()
        val fieldVelocity: ChassisSpeeds = localizer.fieldVelocity
        val linearFieldVelocity = Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond)
        driveController.reset(
            currentPose.translation.getDistance(targetPose().translation),
            min(
                0.0,
                -linearFieldVelocity
                    .rotateBy(targetPose().translation.minus(currentPose.translation).angle.unaryMinus())
                    .x,
            ),
        )
        thetaController.reset(
            currentPose.rotation.rotations,
            Units.radiansToRotations(fieldVelocity.omegaRadiansPerSecond),
        )
        lastSetpointTranslation = currentPose.translation
    }

    override fun execute() {
        running = true

        // Update from tunable numbers
        if (
            driveMaxVelocity.hasChanged(hashCode()) ||
                driveMaxVelocitySlow.hasChanged(hashCode()) ||
                driveMaxAcceleration.hasChanged(hashCode()) ||
                driveTolerance.hasChanged(hashCode()) ||
                thetaMaxVelocity.hasChanged(hashCode()) ||
                thetaMaxAcceleration.hasChanged(hashCode()) ||
                thetaTolerance.hasChanged(hashCode()) ||
                drivekP.hasChanged(hashCode()) ||
                drivekD.hasChanged(hashCode()) ||
                thetakP.hasChanged(hashCode()) ||
                thetakD.hasChanged(hashCode())
        ) {
            driveController.p = drivekP.get()
            driveController.d = drivekD.get()
            driveController.constraints =
                TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get())
            driveController.setTolerance(driveTolerance.get())
            thetaController.p = thetakP.get()
            thetaController.d = thetakD.get()
            thetaController.constraints =
                TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get())
            thetaController.setTolerance(thetaTolerance.get())
        }

        // Get current pose and target pose
        val currentPose = robotPose()
        val targetPose = targetPose()

        // Calculate drive speed
        val currentDistance = currentPose.translation.getDistance(targetPose.translation)
        //        val ffScaler =
        //            MathUtil.clamp((currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() -
        // ffMinRadius.get()), 0.0, 1.0)
        driveErrorAbs = currentDistance
        //        driveController.reset(
        //            lastSetpointTranslation.getDistance(targetPose.translation),
        //            driveController.setpoint.velocity,
        //        )
        var driveVelocityScalar =
            (driveController.setpoint.velocity
            /** ffScaler */
            + driveController.calculate(driveErrorAbs, 0.0))
        if (currentDistance < driveController.positionTolerance) driveVelocityScalar = 0.0
        lastSetpointTranslation =
            Pose2d(targetPose.translation, currentPose.translation.minus(targetPose.translation).angle)
                .transformBy(Transform2d(driveController.setpoint.position, 0.0, Rotation2d.kZero))
                .translation

        // Calculate theta speed
        var thetaVelocity =
            (thetaController.setpoint.velocity
            /** ffScaler */
            + thetaController.calculate(currentPose.rotation.rotations, targetPose.rotation.rotations))
        thetaErrorAbs = abs(currentPose.rotation.minus(targetPose.rotation).rotations)
        if (thetaErrorAbs < thetaController.positionTolerance) thetaVelocity = 0.0

        val driveVelocity =
            Pose2d(Translation2d.kZero, currentPose.translation.minus(targetPose.translation).angle)
                .transformBy(Transform2d(driveVelocityScalar, 0.0, Rotation2d.kZero))
                .translation

        // Command speeds
        drive.setVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.x,
                driveVelocity.y,
                Units.rotationsToRadians(thetaVelocity),
                currentPose.rotation,
            )
        )

        // Log data
        Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance)
        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.setpoint.position)
        Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.rotation.rotations)
        Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.setpoint.position)
        Logger.recordOutput(
            "DriveToPose/Setpoint",
            *arrayOf(Pose2d(lastSetpointTranslation, Rotation2d.fromRotations(thetaController.setpoint.position))),
        )
        Logger.recordOutput("DriveToPose/Goal", *arrayOf(targetPose))
    }

    override fun end(interrupted: Boolean) {
        drive.setVelocity(ChassisSpeeds())
        running = false
        // Clear logs
        Logger.recordOutput("DriveToPose/Setpoint", *arrayOf<Pose2d>())
        Logger.recordOutput("DriveToPose/Goal", *arrayOf<Pose2d>())
    }

    /** Checks if the robot is stopped at the final pose. */
    fun atGoal(): Boolean {
        return running && driveController.atGoal() && thetaController.atGoal()
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    fun withinTolerance(driveTolerance: Double, thetaTolerance: Rotation2d): Boolean {
        return running && abs(driveErrorAbs) < driveTolerance && abs(thetaErrorAbs) < thetaTolerance.rotations
    }

    companion object {
        private val drivekP: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DrivekP")
        private val drivekD: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DrivekD")
        private val thetakP: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/ThetakP")
        private val thetakD: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/ThetakD")
        private val driveMaxVelocity: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DriveMaxVelocity")
        private val driveMaxVelocitySlow: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DriveMaxVelocitySlow")
        private val driveMaxAcceleration: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DriveMaxAcceleration")
        private val thetaMaxVelocity: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/ThetaMaxVelocity")
        private val thetaMaxAcceleration: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration")
        private val driveTolerance: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DriveTolerance")
        private val thetaTolerance: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/ThetaTolerance")
        private val ffMinRadius: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/FFMinRadius")
        private val ffMaxRadius: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/FFMaxRadius")

        init {
            drivekP.initDefault(1.0)
            drivekD.initDefault(0.0)
            thetakP.initDefault(4.0)
            thetakD.initDefault(0.0)
            driveMaxVelocity.initDefault(3.0)
            driveMaxAcceleration.initDefault(3.0)
            thetaMaxVelocity.initDefault(1.0)
            thetaMaxAcceleration.initDefault(2.0)
            driveTolerance.initDefault(0.01)
            thetaTolerance.initDefault(0.002)
        }
    }
}
