package org.team9432.frc2025.robot.subsystems.drive.gyro

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.RadiansPerSecond
import org.ironmaple.simulation.drivesims.GyroSimulation

class GyroIOSim(private val gyroSim: GyroSimulation) : GyroIO {
    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = true
        inputs.odometryYawPositions = gyroSim.cachedGyroReadings
        inputs.yawPosition = gyroSim.gyroReading
        inputs.yawVelocityRadPerSec = gyroSim.measuredAngularVelocity.`in`(RadiansPerSecond)
    }

    override fun setAngle(angle: Rotation2d) {
        gyroSim.setRotation(angle)
    }
}
