package org.team9432.frc2025.robot.subsystems.drive.module

import com.ctre.phoenix6.sim.CANcoderSimState
import com.ctre.phoenix6.sim.ChassisReference
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.RobotController
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.subsystems.drive.ModuleConfig
import org.team9432.frc2025.robot.subsystems.drive.OdometryThread

class ModuleIOSim(private val moduleSim: SwerveModuleSimulation, config: ModuleConfig, odometryThread: OdometryThread) :
    ModuleIOReal(config, odometryThread) {
    private val driveMotor =
        moduleSim.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(DrivetrainConstants.SLIP_CURRENT_AMPS))
    private val steerMotor =
        moduleSim.useGenericControllerForSteer().withCurrentLimit(Amps.of(DrivetrainConstants.STEER_CURRENT_LIMIT_AMPS))

    private val driveSim: TalonFXSimState = super.driveTalon.simState
    private val steerSim: TalonFXSimState = super.steerTalon.simState
    private val cancoderSim: CANcoderSimState = super.cancoder.simState

    init {
        driveSim.Orientation = ChassisReference.CounterClockwise_Positive
        steerSim.Orientation = ChassisReference.Clockwise_Positive
        cancoderSim.Orientation = ChassisReference.CounterClockwise_Positive
    }

    /** Updates the inputs with the latest sensor information. */
    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
        driveSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        steerSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        cancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage())

        driveMotor.requestVoltage(driveSim.motorVoltageMeasure)
        driveSim.setRawRotorPosition(moduleSim.driveWheelFinalPosition * DrivetrainConstants.DRIVE_RATIO)
        driveSim.setRotorVelocity(moduleSim.driveWheelFinalSpeed * DrivetrainConstants.DRIVE_RATIO)

        steerMotor.requestVoltage(steerSim.motorVoltageMeasure)
        steerSim.setRawRotorPosition(moduleSim.steerAbsoluteAngle * DrivetrainConstants.STEER_RATIO)
        steerSim.setRotorVelocity(moduleSim.steerAbsoluteEncoderSpeed * DrivetrainConstants.STEER_RATIO)

        cancoderSim.setRawPosition(moduleSim.steerAbsoluteAngle)
        cancoderSim.setVelocity(moduleSim.steerAbsoluteEncoderSpeed)

        super.updateInputs(inputs)
    }
}
