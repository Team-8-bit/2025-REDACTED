package org.team9432.frc2025.robot.subsystems.rollers.funnel

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.*
import org.team9432.frc2025.lib.util.PhoenixUtil
import org.team9432.frc2025.robot.RobotMap

open class FunnelIOReal : FunnelIO {
    private val talon = TalonFX(RobotMap.funnel.canID, RobotMap.funnel.canBus)

    private val appliedVolts: StatusSignal<Voltage> = talon.motorVoltage
    private val supplyCurrent: StatusSignal<Current> = talon.supplyCurrent
    private val torqueCurrent: StatusSignal<Current> = talon.torqueCurrent
    private val temperature: StatusSignal<Temperature> = talon.deviceTemp
    private val position: StatusSignal<Angle> = talon.position
    private val velocity: StatusSignal<AngularVelocity> = talon.velocity

    private val config =
        TalonFXConfiguration().apply {
            MotorOutput.Inverted = InvertedValue.Clockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Coast

            CurrentLimits.SupplyCurrentLimit = 25.0
            CurrentLimits.SupplyCurrentLimitEnable = true
        }

    init {
        PhoenixUtil.tryUntilOk(5) { talon.configurator.apply(config) }

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            appliedVolts,
            supplyCurrent,
            torqueCurrent,
            temperature,
            position,
            velocity,
        )

        talon.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: FunnelIO.FunnelIOInputs) {
        val status =
            BaseStatusSignal.refreshAll(appliedVolts, supplyCurrent, torqueCurrent, temperature, position, velocity)

        inputs.motorConnected = status.isOK

        inputs.appliedVolts = appliedVolts.valueAsDouble
        inputs.supplyCurrentAmps = supplyCurrent.valueAsDouble
        inputs.torqueCurrentAmps = torqueCurrent.valueAsDouble
        inputs.tempFahrenheit = (temperature.valueAsDouble * (9 / 5)) + 32
        inputs.positionRotations = position.valueAsDouble
        inputs.velocityRotationsPerSec = velocity.valueAsDouble
    }

    override fun setControl(control: ControlRequest) {
        talon.setControl(control)
    }

    override fun updateConfig(block: TalonFXConfiguration.() -> Unit) {
        block.invoke(config)
        PhoenixUtil.tryUntilOk(5) { talon.configurator.apply(config) }
    }
}
