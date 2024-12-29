package org.team9432.frc2025.robot.subsystems.drive.controllers

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team9432.frc2025.lib.AllianceTracker
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.RobotState
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.withSign

class JoystickDriveController(
    private val controllerX: () -> Double,
    private val controllerY: () -> Double,
    private val controllerR: () -> Double,
): DriveController {
    private val ratelimitX = SlewRateLimiter(20.0)
    private val ratelimitY = SlewRateLimiter(20.0)

    private companion object {
        private const val TABLE_KEY = "TeleopDrive"

        private val linearDeadband by LoggedTunableNumber("$TABLE_KEY/LinearDeadband", 0.075)
        private val rotationDeadband by LoggedTunableNumber("$TABLE_KEY/AngularDeadband", 0.0)
    }

    override fun calculate(): ChassisSpeeds {
        val linearSpeed = getLinearSpeed()
        val rotationSpeed = MathUtil.applyDeadband(controllerR(), rotationDeadband)

        val invert = AllianceTracker.switch(blue = 1, red = -1)

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            ratelimitX.calculate(linearSpeed.x * DrivetrainConstants.MAX_LINEAR_SPEED_MPS) * invert,
            ratelimitY.calculate(linearSpeed.y * DrivetrainConstants.MAX_LINEAR_SPEED_MPS) * invert,
            rotationSpeed * DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
            RobotState.currentPose.rotation
        )
    }

    fun getLinearSpeed(): Translation2d {
        val xInput = controllerX()
        val yInput = controllerY()

        // Apply deadband
        var linearMagnitude = MathUtil.applyDeadband(hypot(xInput, yInput), linearDeadband)

        // Get direction of the input
        val linearDirection = Rotation2d(xInput, yInput)

        // Square magnitude
        linearMagnitude = linearMagnitude.pow(2).withSign(linearMagnitude)

        // Calculate new linear velocity
        val linearVelocity =
            Pose2d(0.0, 0.0, linearDirection)
                .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d()))
                .translation

        return linearVelocity
    }
}