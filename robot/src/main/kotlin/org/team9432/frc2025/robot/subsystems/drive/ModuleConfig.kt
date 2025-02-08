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
        Rotation2d.fromRadians(3.55201978322),
        driveMotorInverted = false,
        steerMotorInverted = true,
    ),
    FRONT_RIGHT(
        RobotMap.frontRightDrive,
        RobotMap.frontRightSteer,
        RobotMap.frontRightEncoder,
        Rotation2d.fromRadians(1.372912805157649),
        driveMotorInverted = false,
        steerMotorInverted = true,
    ),
    BACK_LEFT(
        RobotMap.backLeftDrive,
        RobotMap.backLeftSteer,
        RobotMap.backLeftEncoder,
        Rotation2d.fromRadians(1.385184651460734),
        driveMotorInverted = false,
        steerMotorInverted = true,
    ),
    BACK_RIGHT(
        RobotMap.backRightDrive,
        RobotMap.backRightSteer,
        RobotMap.backRightEncoder,
        Rotation2d.fromRadians(.6964272777000811),
        driveMotorInverted = false,
        steerMotorInverted = true,
    ),
}
