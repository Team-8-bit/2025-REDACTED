package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.units.measure.*
import org.team9432.frc2025.lib.util.PhoenixUtil
import org.team9432.frc2025.robot.Constants
import org.team9432.frc2025.robot.RobotMap

open class CoralArmIOReal : CoralArmIO {
    protected val talon = TalonFX(RobotMap.coralArmPivot.canID, RobotMap.coralArmPivot.canBus)

    private val position: StatusSignal<Angle> = talon.position
    private val velocity: StatusSignal<AngularVelocity> = talon.velocity
    private val closedLoopPositionReference: StatusSignal<Double> = talon.closedLoopReference
    private val closedLoopVelocityReference: StatusSignal<Double> = talon.closedLoopReferenceSlope
    private val closedLoopOutput: StatusSignal<Double> = talon.closedLoopOutput

    private val appliedVolts: StatusSignal<Voltage> = talon.motorVoltage
    private val supplyCurrent: StatusSignal<Current> = talon.supplyCurrent
    private val torqueCurrent: StatusSignal<Current> = talon.torqueCurrent
    private val temperature: StatusSignal<Temperature> = talon.deviceTemp

    private val config =
        TalonFXConfiguration().apply {
            Slot0 = Slot0Configs()
            Slot1 = Slot1Configs()

            Slot0.GravityType = GravityTypeValue.Arm_Cosine
            Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign

            MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            TorqueCurrent.PeakForwardTorqueCurrent = CoralArmConstants.PEAK_TORQUE_AMPS
            TorqueCurrent.PeakReverseTorqueCurrent = -CoralArmConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimit = CoralArmConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimitEnable = true

            Feedback.SensorToMechanismRatio = CoralArmConstants.REDUCTION
        }

    init {
        PhoenixUtil.tryUntilOk(5) { talon.configurator.apply(config) }

        BaseStatusSignal.setUpdateFrequencyForAll(200.0, position, velocity)

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            closedLoopPositionReference,
            closedLoopVelocityReference,
            closedLoopOutput,
            appliedVolts,
            supplyCurrent,
            torqueCurrent,
            temperature,
        )

        talon.optimizeBusUtilization()

        if (!Constants.robot.isSim) {
            talon.setPosition(CoralArmConstants.MIN_POSITION)
        }
    }

    override fun updateInputs(inputs: CoralArmIO.CoralArmIOInputs) {
        val status =
            BaseStatusSignal.refreshAll(
                position,
                velocity,
                closedLoopPositionReference,
                closedLoopVelocityReference,
                closedLoopOutput,
                appliedVolts,
                supplyCurrent,
                torqueCurrent,
                temperature,
            )

        inputs.motorConnected = status.isOK

        inputs.positionRotations = position.valueAsDouble
        inputs.velocityRotationsPerSec = velocity.valueAsDouble
        inputs.closedLoopPositionReference = closedLoopPositionReference.valueAsDouble
        inputs.closedLoopVelocityReference = closedLoopVelocityReference.valueAsDouble
        inputs.closedLoopOutput = closedLoopOutput.valueAsDouble

        inputs.appliedVolts = appliedVolts.valueAsDouble
        inputs.supplyCurrentAmps = supplyCurrent.valueAsDouble
        inputs.torqueCurrentAmps = torqueCurrent.valueAsDouble
        inputs.tempFahrenheit = (temperature.valueAsDouble * (9 / 5)) + 32
    }

    override fun setControl(control: ControlRequest) {
        talon.setControl(control)
    }

    override fun updateConfig(block: TalonFXConfiguration.() -> Unit) {
        block.invoke(config)
        PhoenixUtil.tryUntilOk(5) { talon.configurator.apply(config) }
    }

    override fun setBrake(enable: Boolean) {
        Thread { talon.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast) }.start()
    }
}
