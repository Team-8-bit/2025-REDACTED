package org.team9432.frc2025.robot.subsystems.drive.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.*
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.*
import org.team9432.frc2025.lib.util.PhoenixUtil
import org.team9432.frc2025.lib.util.PhoenixUtil.printOnError
import org.team9432.frc2025.robot.subsystems.drive.DrivetrainConstants
import org.team9432.frc2025.robot.subsystems.drive.ModuleConfig
import org.team9432.frc2025.robot.subsystems.drive.OdometryThread
import org.team9432.frc2025.robot.subsystems.drive.module.ModuleIO.ModuleIOInputs
import java.util.*
import java.util.concurrent.Executor
import java.util.concurrent.Executors

class ModuleIOReal(private val config: ModuleConfig, private val odometryThread: OdometryThread): ModuleIO {
    /* Motors & Sensors */
    private val driveTalon = TalonFX(config.driveInformation.canID, config.driveInformation.canBus)
    private val steerTalon = TalonFX(config.steerInformation.canID, config.steerInformation.canBus)
    private val cancoder = CANcoder(config.cancoderInformation.canID, config.cancoderInformation.canBus)

    /* Drive Signals */
    private val driveVelocity: StatusSignal<AngularVelocity> = driveTalon.velocity
    private val driveAppliedVolts: StatusSignal<Voltage> = driveTalon.motorVoltage
    private val driveSupplyCurrent: StatusSignal<Current> = driveTalon.supplyCurrent
    private val driveTorqueCurrent: StatusSignal<Current> = driveTalon.torqueCurrent
    private val driveTemperature: StatusSignal<Temperature> = driveTalon.deviceTemp
    private val driveClosedLoopPositionReference: StatusSignal<Double> = driveTalon.closedLoopReference
    private val driveClosedLoopVelocityReference: StatusSignal<Double> = driveTalon.closedLoopReferenceSlope
    private val driveClosedLoopOutput: StatusSignal<Double> = driveTalon.closedLoopOutput
    private val lowFrequencyDriveSignals =
        arrayOf(driveVelocity, driveAppliedVolts, driveSupplyCurrent, driveTorqueCurrent, driveTemperature, driveClosedLoopPositionReference, driveClosedLoopVelocityReference, driveClosedLoopOutput)

    /* Steer Signals */
    private val steerVelocity: StatusSignal<AngularVelocity> = steerTalon.velocity
    private val steerAppliedVolts: StatusSignal<Voltage> = steerTalon.motorVoltage
    private val steerSupplyCurrent: StatusSignal<Current> = steerTalon.supplyCurrent
    private val steerTorqueCurrent: StatusSignal<Current> = steerTalon.torqueCurrent
    private val steerTemperature: StatusSignal<Temperature> = steerTalon.deviceTemp
    private val steerClosedLoopPositionReference: StatusSignal<Double> = steerTalon.closedLoopReference
    private val steerClosedLoopVelocityReference: StatusSignal<Double> = steerTalon.closedLoopReferenceSlope
    private val steerClosedLoopOutput: StatusSignal<Double> = steerTalon.closedLoopOutput
    private val lowFrequencySteerSignals =
        arrayOf(steerVelocity, steerAppliedVolts, steerSupplyCurrent, steerTorqueCurrent, steerTemperature, driveClosedLoopPositionReference, driveClosedLoopVelocityReference, driveClosedLoopOutput)

    /* CANCoder Signals */
    private val steerAbsolutePosition: StatusSignal<Angle> = cancoder.absolutePosition
    private val lowFrequencyCANCoderSignals = arrayOf(steerAbsolutePosition)

    /* High-frequency Odometry Signals */
    private val drivePosition: StatusSignal<Angle> = driveTalon.position
    private val steerPosition: StatusSignal<Angle> = steerTalon.position
    private val drivePositionQueue: Queue<Double> = odometryThread.registerSignal(drivePosition)
    private val steerPositionQueue: Queue<Double> = odometryThread.registerSignal(steerPosition)
    private val highFrequencySignals = arrayOf(drivePosition, steerPosition)

    /* Control Requests */
    private val voltageControl = VoltageOut(0.0).withUpdateFreqHz(0.0)
    private val currentControl = TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val velocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val positionTorqueCurrentFOC = PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)

    /* Motor Configs */
    private val driveConfig = getDriveConfig()
    private val steerConfig = getSteerConfig()
    private val encoderConfig = getEncoderConfig()
    private val brakeModeExecutor: Executor = Executors.newFixedThreadPool(8)

    /* Connection Debouncers */
    private val driveConnectedDebounce = Debouncer(0.5)
    private val steerConnectedDebounce = Debouncer(0.5)
    private val cancoderConnectedDebounce = Debouncer(0.5)

    init {
        // Configure motors and sensors, also reset the drive motor's position to zero (don't know
        // why it doesn't do this by itself)
        PhoenixUtil.tryUntilOk(5) { driveTalon.configurator.apply(driveConfig, 0.25) }
        PhoenixUtil.tryUntilOk(5) { steerTalon.configurator.apply(steerConfig, 0.25) }
        PhoenixUtil.tryUntilOk(5) { cancoder.configurator.apply(encoderConfig, 0.25) }
        PhoenixUtil.tryUntilOk(5) { driveTalon.setPosition(0.0, 0.25) }

        // Set signal update frequency
        BaseStatusSignal.setUpdateFrequencyForAll(DrivetrainConstants.ODOMETRY_FREQUENCY, *highFrequencySignals)
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            *lowFrequencyDriveSignals,
            *lowFrequencySteerSignals,
            *lowFrequencyCANCoderSignals,
        )

        // Optimize bus utilization
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon, cancoder)
    }

    /** Updates the inputs with the latest sensor information. */
    override fun updateInputs(inputs: ModuleIOInputs) {
        // Refresh signals
        val driveStatus = BaseStatusSignal.refreshAll(*lowFrequencyDriveSignals, drivePosition)
        val steerStatus = BaseStatusSignal.refreshAll(*lowFrequencySteerSignals, steerPosition)
        val cancoderStatus = BaseStatusSignal.refreshAll(*lowFrequencyCANCoderSignals)

        // Debounce connection status to remove false positives, if a device is actually
        // disconnected it will stay that way for a while
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK)
        inputs.steerConnected = steerConnectedDebounce.calculate(steerStatus.isOK)
        inputs.cancoderConnected = cancoderConnectedDebounce.calculate(cancoderStatus.isOK)

        // Update drive inputs
        inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.valueAsDouble)
        inputs.driveVelocityRadPerSecond = Units.rotationsToRadians(driveVelocity.valueAsDouble)
        inputs.driveAppliedVolts = driveAppliedVolts.valueAsDouble
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.valueAsDouble
        inputs.driveTorqueCurrentAmps = driveTorqueCurrent.valueAsDouble
        inputs.driveTempFahrenheit = (driveTemperature.valueAsDouble * (9 / 5)) + 32
        inputs.driveClosedLoopPositionReference = driveClosedLoopPositionReference.valueAsDouble
        inputs.driveClosedLoopVelocityReference = driveClosedLoopVelocityReference.valueAsDouble
        inputs.driveClosedLoopOutput = driveClosedLoopOutput.valueAsDouble

        // Update steer inputs
        inputs.steerAbsolutePosition = Rotation2d.fromRotations(steerAbsolutePosition.valueAsDouble)
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition.valueAsDouble)
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.valueAsDouble)
        inputs.steerAppliedVolts = steerAppliedVolts.valueAsDouble
        inputs.steerSupplyCurrentAmps = steerSupplyCurrent.valueAsDouble
        inputs.steerTorqueCurrentAmps = steerTorqueCurrent.valueAsDouble
        inputs.steerTempFahrenheit = (steerTemperature.valueAsDouble * (9 / 5)) + 32
        inputs.steerClosedLoopPositionReference = steerClosedLoopPositionReference.valueAsDouble
        inputs.steerClosedLoopVelocityReference = steerClosedLoopVelocityReference.valueAsDouble
        inputs.steerClosedLoopOutput = steerClosedLoopOutput.valueAsDouble

        // Update odometry inputs with cached values and reset the queue
        inputs.odometryDrivePositionsRads = drivePositionQueue.map { Units.rotationsToRadians(it) }.toDoubleArray()
        inputs.odometrySteerPositions = steerPositionQueue.map { Rotation2d.fromRotations(it) }.toTypedArray()
        drivePositionQueue.clear()
        steerPositionQueue.clear()
    }

    override fun setDriveControl(control: ControlRequest) {
        driveTalon.setControl(control)
    }

    override fun setSteerControl(control: ControlRequest) {
        steerTalon.setControl(control)
    }

    /** Enables or disables brake mode on the drive motor. */
    override fun setDriveBrake(enable: Boolean) {
        brakeModeExecutor.execute {
            synchronized(driveConfig) {
                driveConfig.MotorOutput.NeutralMode = if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast
                driveTalon.configurator.apply(driveConfig, 0.25)
            }
        }
    }

    /** Enables or disables brake mode on the steer motor. */
    override fun setSteerBrake(enable: Boolean) {
        brakeModeExecutor.execute {
            synchronized(steerConfig) {
                steerConfig.MotorOutput.NeutralMode = if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast
                steerTalon.configurator.apply(steerConfig, 0.25)
            }
        }
    }

    /** Get the drive motor configuration. */
    private fun getDriveConfig() =
        TalonFXConfiguration().apply {
            MotorOutput.Inverted =
                if (config.driveMotorInverted) InvertedValue.Clockwise_Positive
                else InvertedValue.CounterClockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            TorqueCurrent.PeakForwardTorqueCurrent = DrivetrainConstants.SLIP_CURRENT_AMPS
            TorqueCurrent.PeakReverseTorqueCurrent = -DrivetrainConstants.SLIP_CURRENT_AMPS
            CurrentLimits.StatorCurrentLimit = DrivetrainConstants.SLIP_CURRENT_AMPS
            CurrentLimits.StatorCurrentLimitEnable = true

            Feedback.SensorToMechanismRatio = DrivetrainConstants.DRIVE_RATIO
        }

    /** Get the steer motor configuration. */
    private fun getSteerConfig() =
        TalonFXConfiguration().apply {
            MotorOutput.Inverted =
                if (config.steerMotorInverted) InvertedValue.Clockwise_Positive
                else InvertedValue.CounterClockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            TorqueCurrent.PeakForwardTorqueCurrent = DrivetrainConstants.STEER_CURRENT_LIMIT_AMPS
            TorqueCurrent.PeakReverseTorqueCurrent = -DrivetrainConstants.STEER_CURRENT_LIMIT_AMPS
            CurrentLimits.StatorCurrentLimit = DrivetrainConstants.STEER_CURRENT_LIMIT_AMPS
            CurrentLimits.StatorCurrentLimitEnable = true

            Feedback.FeedbackRemoteSensorID = config.cancoderInformation.canID
            Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            Feedback.RotorToSensorRatio = DrivetrainConstants.STEER_RATIO

            ClosedLoopGeneral.ContinuousWrap = true
        }

    /** Get the cancoder configuration. */
    private fun getEncoderConfig() =
        CANcoderConfiguration().apply {
            MagnetSensor.MagnetOffset = config.moduleSensorOffset.rotations
            MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
        }
}
