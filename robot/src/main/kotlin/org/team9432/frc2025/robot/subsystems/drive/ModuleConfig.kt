package org.team9432.frc2025.robot.subsystems.drive

import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.frc2025.lib.util.CANSensorInformation
import org.team9432.frc2025.lib.util.MotorInformation
import org.team9432.frc2025.robot.RobotMap

enum class ModuleConfig(
    val driveInformation: MotorInformation,
    val steerInformation: MotorInformation,
    val cancoderInformation: CANSensorInformation,
    val moduleSensorOffset: Rotation2d,
    val driveMotorInverted: Boolean,
    val steerMotorInverted: Boolean,
) {
    FRONT_LEFT(
        RobotMap.frontLeftDrive,
        RobotMap.frontLeftSteer,
        RobotMap.frontLeftEncoder,
        Rotation2d.fromDegrees(0.0),
        driveMotorInverted = false,
        steerMotorInverted = false,
    ),
    FRONT_RIGHT(
        RobotMap.frontRightDrive,
        RobotMap.frontRightSteer,
        RobotMap.frontRightEncoder,
        Rotation2d.fromDegrees(0.0),
        driveMotorInverted = false,
        steerMotorInverted = false,
    ),
    BACK_LEFT(
        RobotMap.backLeftDrive,
        RobotMap.backLeftSteer,
        RobotMap.backLeftEncoder,
        Rotation2d.fromDegrees(0.0),
        driveMotorInverted = false,
        steerMotorInverted = false,
    ),
    BACK_RIGHT(
        RobotMap.backRightDrive,
        RobotMap.backRightSteer,
        RobotMap.backRightEncoder,
        Rotation2d.fromDegrees(0.0),
        driveMotorInverted = false,
        steerMotorInverted = false,
    ),
}
