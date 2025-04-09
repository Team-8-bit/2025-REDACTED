// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package org.team9432.frc2025.robot.commands.drive

import edu.wpi.first.math.MathUtil
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
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.subsystems.drive.controllers.JoystickDriveController

// By 6328:
// https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/63e5db66eee847360567ef24c3d5807280b300e1/src/main/java/org/littletonrobotics/frc2025/commands/DriveToPose.java
class DriveToPose(
    private val drive: Drive,
    private val localizer: Localizer,
    private val targetPose: () -> Pose2d,
    private val robotPose: () -> Pose2d = { localizer.estimatedPose },
    private val driverInput: JoystickDriveController? = null,
    private val maxVelocityAcceleration: () -> Pair<Double?, Double?> = { null to null },
) : Command() {
    private val driveController = ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(0.0, 0.0))
    private val thetaController = ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(0.0, 0.0))

    private var lastSetpointTranslation = Translation2d()
    private var driveErrorAbs = 0.0
    private var thetaErrorAbs = 0.0

    var running = false
        private set

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

        driveController.constraints = getConstraints()
    }

    private fun getConstraints(): TrapezoidProfile.Constraints {
        val externalInput = maxVelocityAcceleration()
        val velocity = externalInput.first ?: driveMaxVelocity.get()
        val acceleration = externalInput.second ?: driveMaxAcceleration.get()
        return TrapezoidProfile.Constraints(velocity, acceleration)
    }

    override fun execute() {
        running = true

        // Update from tunable numbers
        if (
            driveMaxVelocity.hasChanged(hashCode()) ||
                driveMaxAcceleration.hasChanged(hashCode()) ||
                driveToleranceInches.hasChanged(hashCode()) ||
                thetaMaxVelocity.hasChanged(hashCode()) ||
                thetaMaxAcceleration.hasChanged(hashCode()) ||
                thetaToleranceDegrees.hasChanged(hashCode()) ||
                drivekP.hasChanged(hashCode()) ||
                drivekD.hasChanged(hashCode()) ||
                thetakP.hasChanged(hashCode()) ||
                thetakD.hasChanged(hashCode())
        ) {
            driveController.p = drivekP.get()
            driveController.d = drivekD.get()
            driveController.constraints = TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration())
            driveController.setTolerance(Units.inchesToMeters(driveToleranceInches.get()))
            thetaController.p = thetakP.get()
            thetaController.d = thetakD.get()
            thetaController.constraints =
                TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get())
            thetaController.setTolerance(Units.degreesToRotations(thetaToleranceDegrees.get()))
        }

        // Get current pose and target pose
        val currentPose = robotPose()
        val targetPose = targetPose()

        driveController.constraints = getConstraints()

        // Calculate drive speed
        val currentDistance = currentPose.translation.getDistance(targetPose.translation)
        MathUtil.clamp((currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()), 0.0, 1.0)
        driveErrorAbs = currentDistance
        driveController.reset(
            lastSetpointTranslation.getDistance(targetPose.translation),
            driveController.setpoint.velocity,
        )
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
            (thetaController.setpoint.velocity +
                thetaController.calculate(currentPose.rotation.rotations, targetPose.rotation.rotations))
        thetaErrorAbs = abs(currentPose.rotation.minus(targetPose.rotation).rotations)
        if (thetaErrorAbs < thetaController.positionTolerance) thetaVelocity = 0.0

        val driveVelocity =
            Pose2d(Translation2d.kZero, currentPose.translation.minus(targetPose.translation).angle)
                .transformBy(Transform2d(driveVelocityScalar, 0.0, Rotation2d.kZero))
                .translation

        val driverInput = driverInput?.getSpeeds()

        // Command speeds
        drive.setVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                min(driveVelocity.x + ((driverInput?.first?.x ?: 0.0)), DrivetrainConstants.MAX_LINEAR_SPEED_MPS),
                min(driveVelocity.y + ((driverInput?.first?.y ?: 0.0)), DrivetrainConstants.MAX_LINEAR_SPEED_MPS),
                min(
                    Units.rotationsToRadians(thetaVelocity) + (driverInput?.second ?: 0.0),
                    DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
                ),
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
    fun withinTolerance(driveToleranceInches: Double, thetaToleranceRotations: Double): Boolean {
        return running &&
            abs(driveErrorAbs) < Units.inchesToMeters(driveToleranceInches) &&
            abs(thetaErrorAbs) < thetaToleranceRotations
    }

    companion object {
        private val drivekP: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DrivekP")
        private val drivekD: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DrivekD")
        private val thetakP: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/ThetakP")
        private val thetakD: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/ThetakD")
        private val driveMaxVelocity: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DriveMaxVelocity")
        private val driveMaxAcceleration: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DriveMaxAcceleration")
        private val thetaMaxVelocity: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/ThetaMaxVelocity")
        private val thetaMaxAcceleration: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration")
        private val driveToleranceInches: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/DriveToleranceInches")
        private val thetaToleranceDegrees: LoggedTunableNumber =
            LoggedTunableNumber("DriveToPose/ThetaToleranceDegrees")

        private val ffMinRadius: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/FFMinRadius")
        private val ffMaxRadius: LoggedTunableNumber = LoggedTunableNumber("DriveToPose/FFMaxRadius")

        init {
            drivekP.initDefault(2.0)
            drivekD.initDefault(0.05) // .25
            thetakP.initDefault(4.0) // 6
            thetakD.initDefault(0.4)
            driveMaxVelocity.initDefault(4.0)
            driveMaxAcceleration.initDefault(3.0)
            thetaMaxVelocity.initDefault(1.0) // .5
            thetaMaxAcceleration.initDefault(1.5) // 1
            driveToleranceInches.initDefault(1.0)
            thetaToleranceDegrees.initDefault(1.0)
        }
    }
}
