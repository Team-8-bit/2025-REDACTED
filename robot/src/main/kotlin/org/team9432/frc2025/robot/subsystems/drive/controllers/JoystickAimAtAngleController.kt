package org.team9432.frc2025.robot.subsystems.drive.controllers

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.RobotState
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants

class JoystickAimAtAngleController(
    private val joystickController: JoystickDriveController,
    private val goal: () -> Rotation2d,
    var toleranceDegrees: Double = 1.0,
) : DriveController {
    private val controller =
        ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(0.0, 0.0)).apply {
            enableContinuousInput(-Math.PI, Math.PI)

            reset(
                RobotState.currentPose.rotation.radians,
                RobotState.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond,
            )
        }

    private companion object {
        private const val TABLE_KEY = "TeleopAutoAimController"

        private val kP by LoggedTunableNumber("$TABLE_KEY/kP", 6.0)
        private val kD by LoggedTunableNumber("$TABLE_KEY/kD", 0.3)
        private val maxVelocityMultiplier by LoggedTunableNumber("$TABLE_KEY/MaxVelocityPercent", 0.8)
        private val maxAccelerationMultiplier by LoggedTunableNumber("$TABLE_KEY/MaxAccelerationPercent", 0.7)
    }

    override fun calculate(): ChassisSpeeds {
        controller.setPID(kP, 0.0, kD)
        controller.setTolerance(Units.degreesToRadians(toleranceDegrees))

        val maxAngularVelocity = DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC * maxVelocityMultiplier
        val maxAngularAcceleration = DrivetrainConstants.MAX_LINEAR_ACCEL_MPSPS * maxAccelerationMultiplier
        controller.constraints = TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)

        val controllerOutput = controller.calculate(RobotState.currentPose.rotation.radians, goal.invoke().radians)

        Logger.recordOutput("$TABLE_KEY/PositionErrorDegrees", Units.radiansToDegrees(controller.positionError))

        val joystickSpeeds = joystickController.getLinearSpeed()

        return ChassisSpeeds(joystickSpeeds.x, joystickSpeeds.y, controllerOutput)
    }

    fun atGoal(toleranceDegrees: Double) = abs(Units.radiansToDegrees(controller.positionError)) < toleranceDegrees
}
