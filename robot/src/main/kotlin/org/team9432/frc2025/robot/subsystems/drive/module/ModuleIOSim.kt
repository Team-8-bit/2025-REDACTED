package org.team9432.frc2025.robot.subsystems.drive.module

import edu.wpi.first.units.Units.*
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation

class ModuleIOSim(private val moduleSim: SwerveModuleSimulation) : ModuleIO {
    //    private val driveMotor =
    // moduleSim.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(DrivetrainConstants.SLIP_CURRENT_AMPS))
    //    private val steerMotor =
    // moduleSim.useGenericControllerForSteer().withCurrentLimit(Amps.of(DrivetrainConstants.STEER_CURRENT_LIMIT_AMPS))
    //
    //    private val driveSim: TalonFXSimState = super.driveTalon.simState
    //    private val steerSim: TalonFXSimState = super.steerTalon.simState
    //    private val cancoderSim: CANcoderSimState = super.cancoder.simState
    //
    //    init {
    //        driveSim.Orientation = ChassisReference.Clockwise_Positive
    //        steerSim.Orientation = ChassisReference.Clockwise_Positive
    //        cancoderSim.Orientation = ChassisReference.Clockwise_Positive
    //    }
    //
    //    /** Updates the inputs with the latest sensor information. */
    //    override fun updateInputs(inputs: ModuleIOInputs) {
    //        driveSim.setSupplyVoltage(RobotController.getBatteryVoltage())
    //        steerSim.setSupplyVoltage(RobotController.getBatteryVoltage())
    //        cancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage())
    //
    //        driveMotor.requestVoltage(driveSim.motorVoltageMeasure)
    //        driveSim.setRawRotorPosition(moduleSim.driveMot)
    //
    //        speedSetpoint?.let { targetSpeed ->
    //            val targetVolts = driveFeedback.calculate(inputs.driveVelocityRadPerSecond,
    // targetSpeed) + speedFeedforward
    //            driveMotor.requestVoltage(Volts.of(targetVolts))
    //        }
    //        angleSetpoint?.let { targetAngle ->
    //            val targetVolts = steerFeedback.calculate(inputs.steerAbsolutePosition.radians,
    // targetAngle.radians)
    //            steerMotor.requestVoltage(Volts.of(targetVolts))
    //        }
    //    }
}
