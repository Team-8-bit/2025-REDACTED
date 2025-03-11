package org.team9432.frc2025.robot.subsystems.drive.gyro

import com.ctre.phoenix6.sim.Pigeon2SimState
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotController
import org.ironmaple.simulation.drivesims.GyroSimulation
import org.team9432.frc2025.robot.subsystems.drive.OdometryThread

class GyroIOSim(private val gyroSim: GyroSimulation, odometryThread: OdometryThread) : GyroIOPigeon2(odometryThread) {
    private val pigeonSim: Pigeon2SimState = super.pigeon.simState

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        pigeonSim.setSupplyVoltage(RobotController.getBatteryVoltage())

        pigeonSim.setRawYaw(gyroSim.gyroReading.degrees)
        pigeonSim.setAngularVelocityZ(gyroSim.measuredAngularVelocity)
        //
        //        inputs.connected = true
        //        inputs.odometryYawPositions = gyroSim.cachedGyroReadings
        //        inputs.yawPosition = gyroSim.gyroReading
        //        inputs.yawVelocityRadPerSec =
        // gyroSim.measuredAngularVelocity.`in`(RadiansPerSecond)

        super.updateInputs(inputs)
    }

    override fun setAngle(angle: Rotation2d) {
        gyroSim.setRotation(angle)
    }
}
