package org.team9432.frc2025.robot.commands.drive

import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.team9432.frc2025.robot.RobotState
import org.team9432.frc2025.robot.subsystems.drive.Drive
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import java.text.DecimalFormat
import java.text.NumberFormat
import kotlin.math.abs


class WheelRadiusCharacterization(private val drive: Drive): Command() {
    private val maxAccelDegreesPerSecPerSec = 3.0
    private val maxVelocityDegreesPerSec = 15.0

    private val timer = Timer()

    private val limiter = SlewRateLimiter(Units.degreesToRadians(maxAccelDegreesPerSecPerSec))
    private var initialPositions: Array<Double>? = null
    private var lastAngle = Rotation2d()
    private var accumulatedRotationRadians = 0.0

    override fun initialize() {
        limiter.reset(0.0)
        timer.restart()
    }

    override fun execute() {
        // Run the drivetrain
        val speed = limiter.calculate(Units.degreesToRadians(maxVelocityDegreesPerSec))
        drive.runVelocity(ChassisSpeeds(0.0, 0.0, speed))

        // Leave time for the wheels to orient and the robot to speed up
        if (!timer.hasElapsed(1.0)) return

        // Record initial module positions and robot angle
        if (initialPositions == null) {
            initialPositions = drive.getModuleCharacterizationPositionRads()
            lastAngle = RobotState.currentPose.rotation
        }

        // Record new information
        val rotation = RobotState.currentPose.rotation
        accumulatedRotationRadians += abs((rotation - lastAngle).radians)
        lastAngle = rotation
    }

    override fun end(interrupted: Boolean) {
        // We should do at least one full rotation
        if (accumulatedRotationRadians <= Math.PI * 2.0) {
            println("Not enough data for characterization")
            return
        }

        // Calculate the average wheel distance driven
        val finalPositions = drive.getModuleCharacterizationPositionRads()
        val wheelDistance =
            initialPositions?.zip(finalPositions) // Combine the two lists into one List<Pair<Double, Double>>
                ?.map { (initialPos, finalPos) -> abs(initialPos - finalPos) } // Get the difference between each pair
                ?.average() // Take the average

        // Make sure the wheel initial positions were actually recorded and our wheel distance isn't null
        if (wheelDistance == null) {
            println("Failed to start routine, please wait at LEAST a second before stopping the routine.")
            return
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
        drive.stop()
    }
}
