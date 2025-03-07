package org.team9432.frc2025.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow
import kotlin.math.sqrt
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.AllianceTracker
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants.KINEMATICS
import org.team9432.frc2025.robot.vision.VisionConstants

// By 6328, and with math from the wpilib pose estimator:
// https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/8cd2135a6d7ee105b9f7596bb6261e5d611f4c91/src/main/java/org/littletonrobotics/frc2025/RobotState.java#L301
class Localizer {
    // Must be less than the pose buffer
    private val txTyObservationStaleSecs: LoggedTunableNumber =
        LoggedTunableNumber("RobotState/TxTyObservationStaleSeconds", 0.5)
    private val minDistanceTagPoseBlend: LoggedTunableNumber =
        LoggedTunableNumber("RobotState/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0))
    private val maxDistanceTagPoseBlend: LoggedTunableNumber =
        LoggedTunableNumber("RobotState/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0))

    private val poseBufferSizeSec: Double = 2.0

    private val odometryStateStdDevs: Matrix<N3, N1> = Matrix(VecBuilder.fill(0.003, 0.003, 0.002))

    private val tagPoses2d: Map<Int, Pose2d> =
        VisionConstants.aprilTagLayout.tags.associate { it.ID to it.pose.toPose2d() }

    var odometryPose = Pose2d()
        private set

    var estimatedPose = Pose2d()
        private set

    val rotation: Rotation2d
        get() = estimatedPose.rotation

    private val poseBuffer: TimeInterpolatableBuffer<Pose2d> = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec)

    private val q = Matrix(Nat.N3(), Nat.N1())

    private var lastWheelPositions =
        arrayOf(SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition())

    // Assume gyro starts at zero
    private var gyroOffset = Rotation2d()

    private val txTyPoses: MutableMap<Int, TxTyPoseRecord> = mutableMapOf()

    var robotVelocity = ChassisSpeeds()
    val fieldVelocity
        get() = ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, rotation)

    init {
        for (i in 0..2) {
            q[i, 0] = odometryStateStdDevs[i, 0].pow(2.0)
        }

        for (i in 1..FieldConstants.APRIL_TAG_COUNT) {
            txTyPoses[i] = TxTyPoseRecord(Pose2d(), Double.POSITIVE_INFINITY, -1.0)
        }
    }

    fun resetPose(pose: Pose2d) {
        // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the
        // new frame of rotation
        gyroOffset = pose.rotation.minus(odometryPose.rotation.minus(gyroOffset))
        estimatedPose = pose
        odometryPose = pose
        poseBuffer.clear()
    }

    fun addOdometryObservation(observation: OdometryObservation) {
        val twist: Twist2d = KINEMATICS.toTwist2d(lastWheelPositions, observation.wheelPositions)
        lastWheelPositions = observation.wheelPositions
        val lastOdometryPose = odometryPose
        odometryPose = odometryPose.exp(twist)
        // Use gyro if connected
        observation.gyroAngle?.let { gyroAngle: Rotation2d ->
            // Add offset to measured angle
            val angle = gyroAngle.plus(gyroOffset)
            odometryPose = Pose2d(odometryPose.translation, angle)
        }
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp, odometryPose)
        // Calculate diff from last odometry pose and add onto pose estimate
        val finalTwist = lastOdometryPose.log(odometryPose)
        estimatedPose = estimatedPose.exp(finalTwist)
    }

    fun addVisionObservation(observation: VisionObservation) {
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        if (
            poseBuffer.internalBuffer.isEmpty() ||
                poseBuffer.internalBuffer.lastKey() - poseBufferSizeSec > observation.timestamp
        ) {
            return
        }

        // Get odometry based pose at timestamp
        val sample = poseBuffer.getSample(observation.timestamp).getOrNull() ?: return

        // sample --> odometryPose transform and backwards of that
        val sampleToOdometryTransform = Transform2d(sample, odometryPose)
        val odometryToSampleTransform = Transform2d(odometryPose, sample)
        // get estimate at sample time by applying odometryToSample Transform
        val estimateAtTime = estimatedPose.plus(odometryToSampleTransform)

        val kalmanGains = visionMeasurementStdDevsToKalmanGains(observation.stdDevs)

        // difference between estimate and vision pose
        val transform = Transform2d(estimateAtTime, observation.visionPose)
        // scale transform by visionK
        val kalmanTransform = kalmanGains.times(VecBuilder.fill(transform.x, transform.y, transform.rotation.radians))
        val scaledTransform =
            Transform2d(kalmanTransform[0, 0], kalmanTransform[1, 0], Rotation2d.fromRadians(kalmanTransform[2, 0]))

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform)
    }

    fun addTxTyObservation(observation: TxTyObservation) {
        // Skip if current data for tag is newer
        if (
            txTyPoses.containsKey(observation.tagId) &&
                txTyPoses[observation.tagId]!!.timestamp >= observation.timestamp
        ) {
            return
        }

        // Get rotation at timestamp
        val sample = poseBuffer.getSample(observation.timestamp).getOrNull() ?: return

        val robotRotation = estimatedPose.transformBy(Transform2d(odometryPose, sample)).rotation

        val cameraPose = observation.camera.pose

        // Use 3D distance and tag angles to find robot pose
        val camToTagTranslation =
            Pose3d(Translation3d.kZero, Rotation3d(0.0, observation.ty, -observation.tx))
                .transformBy(Transform3d(Translation3d(observation.distance, 0.0, 0.0), Rotation3d.kZero))
                .translation
                .rotateBy(Rotation3d(0.0, cameraPose.rotation.y, 0.0))
                .toTranslation2d()
        val camToTagRotation = robotRotation.plus(cameraPose.toPose2d().rotation.plus(camToTagTranslation.angle))
        val tagPose2d = tagPoses2d[observation.tagId] ?: return
        val fieldToCameraTranslation =
            Pose2d(tagPose2d.translation, camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(Transform2d(camToTagTranslation.norm, 0.0, Rotation2d.kZero))
                .translation
        var robotPose =
            Pose2d(fieldToCameraTranslation, robotRotation.plus(cameraPose.toPose2d().rotation))
                .transformBy(Transform2d(cameraPose.toPose2d(), Pose2d.kZero))
        // Use gyro angle at time for robot rotation
        robotPose = Pose2d(robotPose.translation, robotRotation)

        // Add transform to current odometry based pose for latency correction
        txTyPoses[observation.tagId] = TxTyPoseRecord(robotPose, camToTagTranslation.norm, observation.timestamp)
    }

    /** Get 2d pose estimate of robot if not stale. */
    fun getTxTyPose(tagId: Int): Pose2d? {
        val cachedPoseData = txTyPoses[tagId] ?: return null
        // Check if stale
        if (Timer.getTimestamp() - cachedPoseData.timestamp >= txTyObservationStaleSecs.get()) {
            return null
        }

        // Get pose at time of cached snapshot
        val sample = poseBuffer.getSample(cachedPoseData.timestamp).getOrNull() ?: return null

        // See how much the robot has moved since then and add it to latency compensate
        val futureDistance = Transform2d(sample, odometryPose)
        return cachedPoseData.pose.plus(futureDistance)
    }

    /**
     * Get estimated pose using txty data given tagId on reef and aligned pose on reef. Used for algae intaking and
     * coral scoring.
     */
    fun getReefPose(face: Int, finalPose: Pose2d): Pose2d {
        val isRed: Boolean = AllianceTracker.currentAlliance == DriverStation.Alliance.Red
        val tagPose =
            getTxTyPose(
                when (face) {
                    1 -> if (isRed) 6 else 19
                    2 -> if (isRed) 11 else 20
                    3 -> if (isRed) 10 else 21
                    4 -> if (isRed) 9 else 22
                    5 -> if (isRed) 8 else 17
                    else -> if (isRed) 7 else 18
                }
            )
        // Use estimated pose if tag pose is not present
        if (tagPose == null) return estimatedPose
        // Use distance from estimated pose to final pose to get t value
        val t =
            MathUtil.clamp(
                (estimatedPose.translation.getDistance(finalPose.translation) - minDistanceTagPoseBlend.get()) /
                    (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
                0.0,
                1.0,
            )
        return estimatedPose.interpolate(tagPose, 1.0 - t)
    }

    private var simulatedPoseSupplier: (() -> Pose2d)? = null

    fun log() {
        Logger.recordOutput("Localizer/OdometryPose", odometryPose)
        Logger.recordOutput("Localizer/EstimatedPose", estimatedPose)

        val tagPoses = Array(FieldConstants.APRIL_TAG_COUNT) { getTxTyPose(it + 1) ?: Pose2d.kZero }
        Logger.recordOutput("Localizer/TxTyPoses", *tagPoses)

        Logger.recordOutput("Localizer/RobotVelocity", robotVelocity)
        Logger.recordOutput("Localizer/FieldVelocity", fieldVelocity)

        simulatedPoseSupplier?.invoke()?.let { Logger.recordOutput("Localizer/SimRobotPosition", it) }
    }

    fun setSimulationPoseSupplier(supplier: () -> Pose2d) {
        simulatedPoseSupplier = supplier
    }

    @JvmRecord
    data class OdometryObservation(
        val wheelPositions: Array<SwerveModulePosition>,
        val gyroAngle: Rotation2d?,
        val timestamp: Double,
    )

    @JvmRecord data class VisionObservation(val visionPose: Pose2d, val timestamp: Double, val stdDevs: Matrix<N3, N1>)

    @JvmRecord
    data class TxTyObservation(
        val tagId: Int,
        val camera: VisionConstants.CameraConstants,
        val tx: Double,
        val ty: Double,
        val distance: Double,
        val timestamp: Double,
    )

    @JvmRecord data class TxTyPoseRecord(val pose: Pose2d, val distance: Double, val timestamp: Double)

    private fun visionMeasurementStdDevsToKalmanGains(visionMeasurementStdDevs: Matrix<N3, N1>): Matrix<N3, N3> {
        val r = DoubleArray(3)
        for (i in 0..2) {
            r[i] = visionMeasurementStdDevs[i, 0] * visionMeasurementStdDevs[i, 0]
        }

        val visionK = Matrix(Nat.N3(), Nat.N3())

        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        for (row in 0..2) {
            if (q[row, 0] == 0.0) {
                visionK[row, row] = 0.0
            } else {
                visionK[row, row] = q[row, 0] / (q[row, 0] + sqrt(q[row, 0] * r[row]))
            }
        }

        return visionK
    }
}
