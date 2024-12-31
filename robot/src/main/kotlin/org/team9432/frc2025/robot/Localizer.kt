package org.team9432.frc2025.robot

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import kotlin.math.max
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants

class Localizer {
    private val poseEstimator: SwerveDrivePoseEstimator =
        SwerveDrivePoseEstimator(
            DrivetrainConstants.KINEMATICS,
            Rotation2d(),
            Array(4) { SwerveModulePosition() },
            Pose2d(),
        )

    private var simulatedPoseSupplier: (() -> Pose2d)? = null
    private var currentChassisSpeeds = ChassisSpeeds()
    private var previousVisionMeasurementTimeStamp: Double = -1.0

    fun applyOdometryObservation(
        currentTimeSeconds: Double,
        gyroRotation: Rotation2d,
        modulePositions: Array<SwerveModulePosition?>,
    ) {
        poseEstimator.updateWithTime(currentTimeSeconds, gyroRotation, modulePositions)
    }

    fun applyVisionMeasurement(visionPose: Pose2d, timestamp: Double, measurementStdDevs: Matrix<N3, N1>) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, measurementStdDevs)
        previousVisionMeasurementTimeStamp = max(timestamp, previousVisionMeasurementTimeStamp)

        Logger.recordOutput("RobotPosition/LatestVisionPose", visionPose)
        Logger.recordOutput("RobotPosition/LatestVisionStddevsXY", measurementStdDevs.get(0, 0))
        Logger.recordOutput("RobotPosition/LatestVisionStddevsRotation", measurementStdDevs.get(2, 0))
        simulatedPoseSupplier?.invoke()?.let { Logger.recordOutput("RobotPosition/ActualSimRobotPosition", it) }
    }

    fun addVelocityData(velocity: ChassisSpeeds) {
        currentChassisSpeeds = velocity
    }

    val currentPose: Pose2d
        get() = poseEstimator.estimatedPosition

    fun resetOdometry(pose: Pose2d, rawGyro: Rotation2d, modulePositions: Array<SwerveModulePosition>) {
        poseEstimator.resetPosition(rawGyro, modulePositions, pose)
    }

    fun getRobotRelativeChassisSpeeds() = currentChassisSpeeds

    fun log() {
        Logger.recordOutput("RobotState/CurrentPose", currentPose)
        Logger.recordOutput("RobotState/CurrentSpeeds", getRobotRelativeChassisSpeeds())
        Logger.recordOutput(
            "RobotState/CurrentFieldRelativeSpeeds",
            ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), currentPose.rotation),
        )
    }

    fun setSimulationPoseSupplier(supplier: () -> Pose2d) {
        simulatedPoseSupplier = supplier
    }
}
