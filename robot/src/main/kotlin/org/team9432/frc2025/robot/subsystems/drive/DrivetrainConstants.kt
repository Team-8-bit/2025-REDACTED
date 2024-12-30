package org.team9432.frc2025.robot.subsystems.drive

import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.system.plant.DCMotor
import org.team9432.frc2025.lib.constants.MK4ISwerveConstants
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Constants

object DrivetrainConstants {
    /** The current at which the drive wheels start to slip, used as a current limit on the drive motors. */
    const val SLIP_CURRENT_AMPS = 60.0

    /** Current limit for the steer motors. */
    const val STEER_CURRENT_LIMIT_AMPS = 40.0

    /** Frequency of signals recorded by drivetrain odometry. */
    const val ODOMETRY_FREQUENCY = 250.0

    const val WHEEL_RADIUS_INCHES = 2.0

    val MODULE_TRANSLATIONS = MK4ISwerveConstants.getModuleTranslationsForFrameSize(26.0)
    val KINEMATICS = SwerveDriveKinematics(*MODULE_TRANSLATIONS)
    val DRIVE_RADIUS = MODULE_TRANSLATIONS[0].norm

    const val MAX_LINEAR_SPEED_MPS = 4.5
    const val MAX_LINEAR_ACCEL_MPSPS = 6.0
    val MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_LINEAR_SPEED_MPS / DRIVE_RADIUS

    const val DRIVE_RATIO = MK4ISwerveConstants.L2PLUS_DRIVE_REDUCTION
    const val STEER_RATIO = MK4ISwerveConstants.STEER_REDUCTION

    val ffkT: Double
    private val ffkS: Double
    private val ffkV: Double
    private val drivekP: Double
    private val drivekD: Double
    private val steerkP: Double
    private val steerkD: Double

    init {
        when (Constants.robot) {
            Constants.RobotType.COMP -> {
                ffkS = 5.0
                ffkV = 0.0
                ffkT = 1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp // A/(N*m)
                drivekP = 35.0
                drivekD = 0.0
                steerkP = 4000.0
                steerkD = 50.0
            }

            Constants.RobotType.SIM -> {
                ffkS = 0.014
                ffkV = 0.134
                ffkT = 0.0
                drivekP = 0.1
                drivekD = 0.0
                steerkP = 10.0
                steerkD = 0.0
            }
        }
    }

    private val tunableDrivekP = LoggedTunableNumber("Drive/Module/DrivekP", drivekP)
    private val tunableDriveKd = LoggedTunableNumber("Drive/Module/DrivekD", drivekD)
    private val tunableDrivekS = LoggedTunableNumber("Drive/Module/DrivekS", ffkS)
    private val tunableDrivekV = LoggedTunableNumber("Drive/Module/DrivekV", ffkV)
    private val tunableSteerkP = LoggedTunableNumber("Drive/Module/SteerkP", steerkP)
    private val tunableSteerkD = LoggedTunableNumber("Drive/Module/SteerkD", steerkD)

    fun checkDriveFFChange(id: Int, action: (Double, Double) -> Unit) = LoggedTunableNumber.ifChanged(id, tunableDrivekS, tunableDrivekV) { (kS, kV) -> action.invoke(kS, kV) }

    fun checkDrivePidChange(id: Int, action: (Double, Double) -> Unit) = LoggedTunableNumber.ifChanged(id, tunableDrivekP, tunableDriveKd) { (kP, kD) -> action.invoke(kP, kD) }

    fun checkSteerPidChange(id: Int, action: (Double, Double) -> Unit) = LoggedTunableNumber.ifChanged(id, tunableSteerkP, tunableSteerkD) { (kP, kD) -> action.invoke(kP, kD) }
}
