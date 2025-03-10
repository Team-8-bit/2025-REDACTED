package org.team9432.frc2025.robot.subsystems.drive.controllers

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Localizer

class JoystickAimAtAngleController(
    private val joystickController: JoystickDriveController,
    private val goal: () -> Rotation2d,
    private val localizer: Localizer,
    var toleranceDegrees: Double = 1.0,
) : DriveController {
    private val controller =
        ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(0.0, 0.0)).apply {
            enableContinuousInput(-0.5, 0.5)

            reset(localizer.rotation.rotations, Units.radiansToRotations(localizer.robotVelocity.omegaRadiansPerSecond))
        }

    private companion object {
        private const val TABLE_KEY = "TeleopAutoAimController"

        private val kP by LoggedTunableNumber("$TABLE_KEY/kP", 6.0)
        private val kD by LoggedTunableNumber("$TABLE_KEY/kD", 0.3)
        private val maxVelocity by LoggedTunableNumber("$TABLE_KEY/MaxVelocityRotationsPerSec", 1.0)
        private val maxAcceleration by LoggedTunableNumber("$TABLE_KEY/MaxAccelerationRotationsPerSecPerSec", 2.0)
    }

    override fun calculate(): ChassisSpeeds {
        controller.setPID(kP, 0.0, kD)
        controller.setTolerance(Units.degreesToRotations(toleranceDegrees))

        val maxAngularVelocity = maxVelocity
        val maxAngularAcceleration = maxAcceleration
        controller.constraints = TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)

        val controllerOutput = controller.calculate(localizer.rotation.rotations, goal.invoke().rotations)

        Logger.recordOutput("$TABLE_KEY/PositionErrorDegrees", Units.rotationsToDegrees(controller.positionError))

        val joystickSpeeds = joystickController.getSpeeds().first
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            joystickSpeeds.x,
            joystickSpeeds.y,
            controllerOutput,
            localizer.rotation,
        )
    }

    fun atGoal(toleranceDegrees: Double) = abs(Units.rotationsToDegrees(controller.positionError)) < toleranceDegrees
}
