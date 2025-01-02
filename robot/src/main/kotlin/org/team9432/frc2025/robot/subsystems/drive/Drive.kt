package org.team9432.frc2025.robot.subsystems.drive

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock
import kotlin.math.min
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.subsystems.drive.controllers.DriveController
import org.team9432.frc2025.robot.subsystems.drive.gyro.GyroIO
import org.team9432.frc2025.robot.subsystems.drive.gyro.LoggedGyroIOInputs
import org.team9432.frc2025.robot.subsystems.drive.module.ModuleIO
import org.team9432.frc2025.robot.subsystems.drive.module.SwerveModule

class Drive(
    private val localizer: Localizer,
    private val gyroIO: GyroIO,
    frontLeft: ModuleIO,
    frontRight: ModuleIO,
    backLeft: ModuleIO,
    backRight: ModuleIO,
    private val odometryThread: OdometryThread,
) : SubsystemBase() {
    private val gyroInputs = LoggedGyroIOInputs()
    private val odometryThreadInputs = LoggedOdometryThreadInputs()

    private val modules: Array<SwerveModule> =
        arrayOf(
            SwerveModule(frontLeft, "Front Left"),
            SwerveModule(frontRight, "Front Right"),
            SwerveModule(backLeft, "Back Left"),
            SwerveModule(backRight, "Back Right"),
        )

    private val gyroDisconnectedAlert = Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError)

    private var rawGyroRotation = Rotation2d()
    private val lastModulePositions = Array(4) { SwerveModulePosition() }

    companion object {
        val odometryLock = ReentrantLock()
    }

    override fun periodic() {
        // Update odometry
        odometryLock.withLock {
            gyroIO.updateInputs(gyroInputs)
            Logger.processInputs("Drive/Gyro", gyroInputs)

            odometryThread.updateInputs(odometryThreadInputs)
            Logger.processInputs("Drive/OdometryThread", odometryThreadInputs)

            modules.forEach(SwerveModule::updateInputs)
        }

        Logger.recordOutput("SwerveStates/Measured", *getModuleStates())

        if (DriverStation.isDisabled()) {
            runVelocity(ChassisSpeeds())
        }

        // On the real robot these will be the same because the module's sensor samples are recorded
        // at the same time as the timestamps
        // However, in simulation we don't actually have motor sensors to read from so they aren't
        // necessarily the same
        val minSamples = min(odometryThreadInputs.timestamps.size, modules[0].odometrySampleSize)

        for (timestampIndex in 0..<minSamples) {
            val modulePositions = arrayOfNulls<SwerveModulePosition>(4)
            val moduleDeltas = arrayOfNulls<SwerveModulePosition>(4)

            for (moduleIndex in modules.indices) {
                val currentPosition = modules[moduleIndex].getOdometryModulePositions()[timestampIndex]
                modulePositions[moduleIndex] = currentPosition
                moduleDeltas[moduleIndex] =
                    SwerveModulePosition(
                        currentPosition.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        currentPosition.angle,
                    )
                lastModulePositions[moduleIndex] = currentPosition
            }

            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[timestampIndex]
            } else {
                val twist = DrivetrainConstants.KINEMATICS.toTwist2d(*moduleDeltas)
                rawGyroRotation += Rotation2d(twist.dtheta)
            }

            localizer.applyOdometryObservation(
                odometryThreadInputs.timestamps[timestampIndex],
                rawGyroRotation,
                modulePositions,
            )
        }

        // Add velocity data
        val robotRelativeSpeeds = DrivetrainConstants.KINEMATICS.toChassisSpeeds(*getModuleStates())
        if (gyroInputs.connected) {
            robotRelativeSpeeds.omegaRadiansPerSecond = gyroInputs.yawVelocityRadPerSec
        }
        localizer.addVelocityData(robotRelativeSpeeds)

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected)
    }

    fun controllerCommand(controller: DriveController): Command = run { runVelocity(controller.calculate()) }

    private val zeroSwerveModuleState = SwerveModuleState()

    fun runVelocity(speeds: ChassisSpeeds, torqueFF: Array<SwerveModuleState>? = null) {
        // Calculate module setpoints
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        val setpointStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DrivetrainConstants.MAX_LINEAR_SPEED_MPS)

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds)

        // Send setpoints to modules
        for (i in modules.indices) {
            val feedforward = torqueFF?.get(i) ?: zeroSwerveModuleState
            val setpoint = setpointStates[i]
            setpoint.optimize(modules[i].angle)
            setpoint.cosineScale(modules[i].angle)
            modules[i].runSetpoint(setpoint, feedforward)
        }

        // Log modified optimized setpoints
        Logger.recordOutput("SwerveStates/SetpointsOptimized", *setpointStates)
    }

    /** Stops the drivetrain. Equivalent to running `runVelocity(ChassisSpeeds())`. */
    fun stop() = runVelocity(ChassisSpeeds())

    /** Sets the steer motors to the given setpoints and applies the given number of amps to the drive motors. */
    fun runDriveCharacterizationVoltage(voltage: Double, steerSetpoints: Array<Rotation2d>) {
        for ((index, module) in modules.withIndex()) {
            module.runDriveCharacterizationVoltage(voltage, steerSetpoints[index])
        }
    }

    /** Returns the module states of the modules. */
    fun getModuleStates() = Array(modules.size) { modules[it].measuredState }

    /** Returns the module positions of the modules. */
    fun getModulePositions() = Array(modules.size) { modules[it].measuredPosition }

    /** Returns the positions of each wheel in radians. */
    fun getModuleCharacterizationPositionRads() = Array(modules.size) { modules[it].characterizationWheelPositionRads }
}
