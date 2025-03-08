package org.team9432.frc2025.robot.subsystems.drive.module

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.Constants
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants

class SwerveModule(private val io: ModuleIO, private val name: String) {
    private val inputs: LoggedModuleIOInputs = LoggedModuleIOInputs()

    private val driveDisconnectedAlert = Alert("$name drive disconnected!", AlertType.kError)
    private val steerDisconnectedAlert = Alert("$name steer disconnected!", AlertType.kError)
    private val cancoderDisconnectedAlert = Alert("$name cancoder disconnected!", AlertType.kError)

    private val voltageControl = VoltageOut(0.0)
    private val currentControl = TorqueCurrentFOC(0.0)
    private val velocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)
    private val motionMagicPositionControl = MotionMagicTorqueCurrentFOC(0.0)

    private val gains: TunableModuleGains

    var goal: SwerveModuleState = SwerveModuleState()
    var torqueFF: Double? = null

    var coastOverride = { false }
    private var wasCoast = false

    var characterizationInput: Double? = null
    var characterizationAngle: Rotation2d? = null

    init {
        io.setDriveBrake(true)
        io.setSteerBrake(true)

        gains =
            when (Constants.robot) {
                Constants.RobotType.COMP -> {
                    TunableModuleGains(
                        "Drive/ModuleGains",
                        kPDrive = 40.0,
                        kDDrive = 0.0,
                        kSDrive = 12.0,
                        kVDrive = 1.4,
                        kPSteer = 2500.0,
                        kDSteer = 70.0,
                        mmCruiseSteer = 8.0,
                        mmAccelSteer = 28.0,
                    )
                }

                Constants.RobotType.SIM -> {
                    TunableModuleGains(
                        "Drive/ModuleGains",
                        kPDrive = 0.0,
                        kDDrive = 0.0,
                        kSDrive = 0.0,
                        kVDrive = 0.0,
                        kPSteer = 0.0,
                        kDSteer = 0.0,
                        mmCruiseSteer = 0.0,
                        mmAccelSteer = 0.0,
                    )
                }
            }
    }

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Drive/${name}", inputs)

        driveDisconnectedAlert.set(!inputs.driveConnected)
        steerDisconnectedAlert.set(!inputs.steerConnected)
        cancoderDisconnectedAlert.set(!inputs.cancoderConnected)

        gains.ifDriveChanged(hashCode()) { io.updateDriveConfig { config -> gains.applyToDriveConfig(config) } }
        gains.ifSteerChanged(hashCode()) { io.updateSteerConfig { config -> gains.applyToSteerConfig(config) } }

        val shouldCoast = coastOverride()
        val shouldRunClosedLoop = characterizationInput == null && characterizationAngle == null && !shouldCoast

        if (shouldRunClosedLoop) {
            val torqueFeedforward =
                torqueFF?.let { (it / DrivetrainConstants.DRIVE_RATIO) * DrivetrainConstants.ffkT } ?: 0.0
            io.setDriveControl(
                velocityTorqueCurrentFOC
                    .withVelocity(goal.speedMetersPerSecond / DrivetrainConstants.WHEEL_CIRCUMFERENCE)
                    .withFeedForward(torqueFeedforward)
            )
            io.setSteerControl(motionMagicPositionControl.withPosition(goal.angle.rotations))
        } else if (characterizationInput != null || characterizationAngle != null) {
            io.setDriveControl(currentControl.withOutput(characterizationInput ?: 0.0))
            io.setSteerControl(
                motionMagicPositionControl.withPosition(characterizationAngle?.rotations ?: angle.rotations)
            )
        }

        if (shouldCoast != wasCoast) {
            wasCoast = shouldCoast
            io.setDriveBrake(!shouldCoast)
            io.setSteerBrake(!shouldCoast)
        }

        Logger.recordOutput("Drive/${name}/ModuleAngleRotations", angle.rotations)
        Logger.recordOutput("Drive/${name}/CharacterizationInputAngle", characterizationAngle?.rotations ?: 0.0)
        Logger.recordOutput("Drive/${name}/CharacterizationInputAmps", characterizationInput ?: 0.0)
    }

    /** Get an array of the module positions recorded in the last call to [periodic]. */
    val odometryModulePositions: Array<SwerveModulePosition>
        get() =
            Array(odometrySampleSize) { index ->
                val positionMeters =
                    inputs.odometryDrivePositionsRotations[index] * DrivetrainConstants.WHEEL_CIRCUMFERENCE
                val angle = inputs.odometrySteerPositions[index]
                SwerveModulePosition(positionMeters, angle)
            }

    /** The current angle of the module. */
    val angle
        get() = inputs.steerAbsolutePosition

    /** Current module state as reported by the robot's sensors. */
    val measuredState
        get() =
            SwerveModuleState(inputs.driveVelocityRotationsPerSecond * DrivetrainConstants.WHEEL_CIRCUMFERENCE, angle)

    /** Current module position as reported by the robot's sensors. */
    val measuredPosition
        get() = SwerveModulePosition(inputs.drivePositionRotations * DrivetrainConstants.WHEEL_CIRCUMFERENCE, angle)

    val wheelPositionRotations
        get() = inputs.drivePositionRotations

    val wheelVelocityRotationsPerSecond
        get() = inputs.driveVelocityRotationsPerSecond

    /** The number of cached odometry readings. */
    val odometrySampleSize
        get() = minOf(inputs.odometryDrivePositionsRotations.size, inputs.odometrySteerPositions.size)
}
