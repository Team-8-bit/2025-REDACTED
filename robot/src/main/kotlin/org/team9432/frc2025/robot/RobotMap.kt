package org.team9432.frc2025.robot

import com.ctre.phoenix6.CANBus
import org.team9432.frc2025.lib.util.CANSensorInformation
import org.team9432.frc2025.lib.util.MotorInformation

object RobotMap {
    const val DRIVETRAIN_CANBUS_NAME = "Drivetrain"
    val drivetrainCanbus = CANBus(DRIVETRAIN_CANBUS_NAME)

    val frontLeftDrive = MotorInformation(canID = 1, pdhChannel = 1, DRIVETRAIN_CANBUS_NAME)
    val frontLeftSteer = MotorInformation(canID = 2, pdhChannel = 2, DRIVETRAIN_CANBUS_NAME)
    val frontRightDrive = MotorInformation(canID = 3, pdhChannel = 3, DRIVETRAIN_CANBUS_NAME)
    val frontRightSteer = MotorInformation(canID = 4, pdhChannel = 4, DRIVETRAIN_CANBUS_NAME)
    val backLeftDrive = MotorInformation(canID = 5, pdhChannel = 5, DRIVETRAIN_CANBUS_NAME)
    val backLeftSteer = MotorInformation(canID = 6, pdhChannel = 6, DRIVETRAIN_CANBUS_NAME)
    val backRightDrive = MotorInformation(canID = 7, pdhChannel = 7, DRIVETRAIN_CANBUS_NAME)
    val backRightSteer = MotorInformation(canID = 8, pdhChannel = 8, DRIVETRAIN_CANBUS_NAME)

    val frontLeftEncoder = CANSensorInformation(canID = 1, DRIVETRAIN_CANBUS_NAME)
    val frontRightEncoder = CANSensorInformation(canID = 2, DRIVETRAIN_CANBUS_NAME)
    val backLeftEncoder = CANSensorInformation(canID = 3, DRIVETRAIN_CANBUS_NAME)
    val backRightEncoder = CANSensorInformation(canID = 4, DRIVETRAIN_CANBUS_NAME)

    val pigeon = CANSensorInformation(canID = 1, DRIVETRAIN_CANBUS_NAME)
}