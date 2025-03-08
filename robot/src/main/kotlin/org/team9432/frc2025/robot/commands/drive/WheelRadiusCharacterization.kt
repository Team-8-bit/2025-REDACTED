package org.team9432.frc2025.robot.commands.drive

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import java.text.DecimalFormat
import java.text.NumberFormat
import kotlin.math.abs
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.subsystems.drive.Drive
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants

fun WheelRadiusCharacterization(drive: Drive, localizer: Localizer): Command {
    val maxAccelDegreesPerSecPerSec = 3.0
    val maxVelocityDegreesPerSec = 15.0

    val timer = Timer()

    val limiter = SlewRateLimiter(Units.degreesToRadians(maxAccelDegreesPerSecPerSec))
    var initialPositions: List<Double>? = null
    var lastAngle = Rotation2d()
    var accumulatedRotationRadians = 0.0

    return Commands.runOnce({
            limiter.reset(0.0)
            timer.restart()
        })
        .andThen(
            drive.runVelocity({
                // Run the drivetrain
                val speed = limiter.calculate(Units.degreesToRadians(maxVelocityDegreesPerSec))

                // Leave time for the wheels to orient and the robot to speed up
                if (!timer.hasElapsed(1.0)) return@runVelocity ChassisSpeeds()

                // Record initial module positions and robot angle
                if (initialPositions == null) {
                    initialPositions =
                        drive.getModuleCharacterizationPositionRotations().map { Units.rotationsToRadians(it) }
                    lastAngle = localizer.rotation
                }

                // Record new information
                val rotation = localizer.rotation
                accumulatedRotationRadians += abs((rotation - lastAngle).radians)
                lastAngle = rotation

                return@runVelocity ChassisSpeeds(0.0, 0.0, speed)
            })
        )
        .finallyDo { interrupted ->
            // We should do at least one full rotation
            if (accumulatedRotationRadians <= Math.PI * 2.0) {
                println("Not enough data for characterization")
                return@finallyDo
            }

            // Calculate the average wheel distance driven
            val finalPositions = drive.getModuleCharacterizationPositionRotations().map { Units.rotationsToRadians(it) }
            val wheelDistance =
                initialPositions
                    ?.zip(finalPositions) // Combine the two lists into one List<Pair<Double, Double>>
                    ?.map { (initialPos, finalPos) ->
                        abs(initialPos - finalPos)
                    } // Get the difference between each pair
                    ?.average() // Take the average

            // Make sure the wheel initial positions were actually recorded and our wheel distance
            // isn't
            // null
            if (wheelDistance == null) {
                println("Failed to start routine, please wait at LEAST a second before stopping the routine.")
                return@finallyDo
            }

            // Calculate wheel radius
            val wheelRadius = (accumulatedRotationRadians * DrivetrainConstants.DRIVE_RADIUS) / wheelDistance

            // Print output
            val formatter: NumberFormat = DecimalFormat("#0.000")
            println("********** Wheel Radius Characterization Results **********")
            println("Wheel Delta: ${formatter.format(wheelDistance)} radians")
            println("Gyro Delta: ${formatter.format(accumulatedRotationRadians)} radians")
            println(("Wheel Radius: ${formatter.format(Units.metersToInches(wheelRadius))} inches"))

            // Stop timer and robot
            timer.stop()
        }
}
