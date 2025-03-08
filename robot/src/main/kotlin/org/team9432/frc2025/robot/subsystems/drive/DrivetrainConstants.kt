package org.team9432.frc2025.robot.subsystems.drive

import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import org.team9432.frc2025.lib.constants.MK4NSwerveConstants

object DrivetrainConstants {
    /** The current at which the drive wheels start to slip, used as a current limit on the drive motors. */
    const val SLIP_CURRENT_AMPS = 60.0

    /** Current limit for the steer motors. */
    const val STEER_CURRENT_LIMIT_AMPS = 40.0

    /** Frequency of signals recorded by drivetrain odometry. */
    const val ODOMETRY_FREQUENCY = 250.0

    val WHEEL_RADIUS = Units.inchesToMeters(2.0)
    val WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * Math.PI * 2.0

    val MODULE_TRANSLATIONS = MK4NSwerveConstants.getModuleTranslationsForFrameSize(29.5)
    val KINEMATICS = SwerveDriveKinematics(*MODULE_TRANSLATIONS)
    val DRIVE_RADIUS = MODULE_TRANSLATIONS[0].norm

    const val MAX_LINEAR_SPEED_MPS = 4.5
    const val MAX_LINEAR_ACCEL_MPSPS = 6.0
    val MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_LINEAR_SPEED_MPS / DRIVE_RADIUS

    const val DRIVE_RATIO = MK4NSwerveConstants.L1PLUS_DRIVE_REDUCTION
    const val STEER_RATIO = MK4NSwerveConstants.STEER_REDUCTION

    val ffkT: Double = 1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp // A/(N*m)
}
