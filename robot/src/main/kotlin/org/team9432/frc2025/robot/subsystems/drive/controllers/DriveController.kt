package org.team9432.frc2025.robot.subsystems.drive.controllers

import edu.wpi.first.math.kinematics.ChassisSpeeds

fun interface DriveController {
    fun calculate(): ChassisSpeeds
}