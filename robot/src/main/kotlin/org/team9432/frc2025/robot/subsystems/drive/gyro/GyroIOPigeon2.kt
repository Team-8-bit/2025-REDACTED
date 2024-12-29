package org.team9432.frc2025.robot.subsystems.drive.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import java.util.*
import org.team9432.frc2025.robot.RobotMap
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.subsystems.drive.OdometryThread

/** IO implementation for Pigeon2 */
class GyroIOPigeon2 : GyroIO {
    private val pigeon = Pigeon2(RobotMap.pigeon.canID, RobotMap.pigeon.canBus)
    private val yaw: StatusSignal<Angle> = pigeon.yaw
    private var yawPositionQueue: Queue<Double>
    private val yawVelocity: StatusSignal<AngularVelocity> = pigeon.angularVelocityZWorld

    init {
        pigeon.configurator.apply(Pigeon2Configuration())
        pigeon.configurator.setYaw(0.0)

        yawVelocity.setUpdateFrequency(100.0)
        yaw.setUpdateFrequency(DrivetrainConstants.ODOMETRY_FREQUENCY)

        yawPositionQueue = OdometryThread.registerSignal(yaw)

        pigeon.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.valueAsDouble)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.valueAsDouble)

        inputs.odometryYawPositions = yawPositionQueue.map { Rotation2d.fromDegrees(it) }.toTypedArray()
        yawPositionQueue.clear()
    }

    override fun setAngle(angle: Rotation2d) {
        pigeon.setYaw(angle.degrees)
    }
}
