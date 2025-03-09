package org.team9432.frc2025.robot.subsystems.drive

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
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
    private val gyroIO: GyroIO,
    frontLeft: ModuleIO,
    frontRight: ModuleIO,
    backLeft: ModuleIO,
    backRight: ModuleIO,
    private val odometryThread: OdometryThread,
    private val localizer: Localizer,
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

    companion object {
        val odometryLock = ReentrantLock()
    }

    var coastOverride = { false }
        set(value) {
            modules.forEach { it.coastOverride = value }
            field = value
        }

    override fun periodic() {
        // Update odometry
        odometryLock.withLock {
            gyroIO.updateInputs(gyroInputs)
            Logger.processInputs("Drive/Gyro", gyroInputs)

            odometryThread.updateInputs(odometryThreadInputs)
            Logger.processInputs("Drive/OdometryThread", odometryThreadInputs)

            modules.forEach(SwerveModule::periodic)
        }

        Logger.recordOutput("Drive/SwerveStates/Measured", *getModuleStates())

        // On the real robot these will be the same because the module's sensor samples are recorded
        // at the same time as the timestamps
        // However, in simulation we don't actually have motor sensors to read from so they aren't
        // necessarily the same
        val minSamples = min(odometryThreadInputs.timestamps.size, modules[0].odometrySampleSize)

        for (timestampIndex in 0..<minSamples) {
            val modulePositions = Array(modules.size) { modules[it].odometryModulePositions[timestampIndex] }

            localizer.addOdometryObservation(
                Localizer.OdometryObservation(
                    modulePositions,
                    if (gyroInputs.connected) gyroInputs.odometryYawPositions[timestampIndex] else null,
                    odometryThreadInputs.timestamps[timestampIndex],
                )
            )
        }

        // Add velocity data
        val robotRelativeSpeeds = DrivetrainConstants.KINEMATICS.toChassisSpeeds(*getModuleStates())

        localizer.robotVelocity = robotRelativeSpeeds

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected)
    }

    fun resetGyro() {
        gyroIO.setAngle(Rotation2d())
    }

    fun controllerCommand(controller: DriveController): Command = runVelocity(controller::calculate)

    fun runVelocity(speed: ChassisSpeeds, torqueFF: Array<Double>? = null) =
        runVelocity({ speed }, torqueFF?.let { { it } })

    fun runVelocity(speedSupplier: () -> ChassisSpeeds, torqueFF: (() -> Array<Double>)? = null): Command = run {
        setVelocity(speedSupplier.invoke(), torqueFF?.invoke())
    }

    fun setVelocity(speed: ChassisSpeeds, torqueFF: Array<Double>? = null) {
        // Calculate module setpoints
        val discreteSpeeds = ChassisSpeeds.discretize(speed, 0.02)
        val setpointStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DrivetrainConstants.MAX_LINEAR_SPEED_MPS)

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Drive/SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("Drive/RunVelocitySpeeds", discreteSpeeds)

        // Send setpoints to modules
        for (i in modules.indices) {
            val feedforward = torqueFF?.get(i) ?: 0.0
            val setpoint = setpointStates[i]
            setpoint.optimize(modules[i].angle)
            setpoint.cosineScale(modules[i].angle)
            modules[i].goal = setpoint
            modules[i].torqueFF = feedforward
        }

        // Log modified optimized setpoints
        Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", *setpointStates)
    }

    /** Sets the steer motors to the given setpoints and applies the given number of amps to the drive motors. */
    fun runDriveCharacterizationAmperage(amps: () -> Double, steerSetpoints: Array<Rotation2d>) =
        runEnd(
            {
                for ((index, module) in modules.withIndex()) {
                    module.characterizationInput = amps.invoke()
                    module.characterizationAngle = steerSetpoints[index]
                }
            },
            {
                for (module in modules) {
                    module.characterizationInput = null
                    module.characterizationAngle = null
                }
            },
        )

    /** Returns the module states of the modules. */
    fun getModuleStates() = Array(modules.size) { modules[it].measuredState }

    /** Returns the module positions of the modules. */
    fun getModulePositions() = Array(modules.size) { modules[it].measuredPosition }

    /** Returns the positions of each wheel in rotations. */
    fun getModuleCharacterizationPositionRotations() = Array(modules.size) { modules[it].wheelPositionRotations }

    fun getModuleCharacterizationVelocityRotationsPerSecond() =
        Array(modules.size) { modules[it].wheelPositionRotations }
}
