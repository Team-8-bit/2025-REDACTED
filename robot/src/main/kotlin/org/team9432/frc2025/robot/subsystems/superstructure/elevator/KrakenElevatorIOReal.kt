package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.*
import org.team9432.frc2025.lib.util.PhoenixUtil
import org.team9432.frc2025.robot.RobotMap

open class KrakenElevatorIOReal : KrakenElevatorIO {
    protected val motor = TalonFX(RobotMap.leftElevator.canID, RobotMap.leftElevator.canBus)
    protected val follower = TalonFX(RobotMap.rightElevator.canID, RobotMap.rightElevator.canBus)

    private val leaderPosition: StatusSignal<Angle> = motor.position
    private val leaderVelocity: StatusSignal<AngularVelocity> = motor.velocity
    private val leaderAppliedVolts: StatusSignal<Voltage> = motor.motorVoltage
    private val leaderSupplyCurrent: StatusSignal<Current> = motor.supplyCurrent
    private val leaderTorqueCurrent: StatusSignal<Current> = motor.torqueCurrent
    private val leaderTemperature: StatusSignal<Temperature> = motor.deviceTemp

    private val followerPosition: StatusSignal<Angle> = follower.position
    private val followerVelocity: StatusSignal<AngularVelocity> = follower.velocity
    private val followerAppliedVolts: StatusSignal<Voltage> = follower.motorVoltage
    private val followerSupplyCurrent: StatusSignal<Current> = follower.supplyCurrent
    private val followerTorqueCurrent: StatusSignal<Current> = follower.torqueCurrent
    private val followerTemperature: StatusSignal<Temperature> = follower.deviceTemp

    private val closedLoopPositionReference: StatusSignal<Double> = motor.closedLoopReference
    private val closedLoopVelocityReference: StatusSignal<Double> = motor.closedLoopReferenceSlope
    private val closedLoopOutput: StatusSignal<Double> = motor.closedLoopOutput

    init {
        follower.setControl(Follower(RobotMap.leftElevator.canID, /* OpposeMasterDirection= */ true))

        BaseStatusSignal.setUpdateFrequencyForAll(
            200.0,
            leaderPosition,
            leaderVelocity,
            followerPosition,
            followerVelocity,
        )

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            followerAppliedVolts,
            followerSupplyCurrent,
            followerTorqueCurrent,
            followerTemperature,
            leaderAppliedVolts,
            leaderSupplyCurrent,
            leaderTorqueCurrent,
            leaderTemperature,
            closedLoopPositionReference,
            closedLoopVelocityReference,
            closedLoopOutput,
        )

        motor.optimizeBusUtilization(0.0, 1.0)
        follower.optimizeBusUtilization(0.0, 1.0)
    }

    override fun updateInputs(inputs: KrakenElevatorIO.ElevatorIOInputs) {
        val leaderStatus =
            BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderAppliedVolts,
                leaderSupplyCurrent,
                leaderTorqueCurrent,
                leaderTemperature,
                closedLoopPositionReference,
                closedLoopVelocityReference,
                closedLoopOutput,
            )
        val followerStatus =
            BaseStatusSignal.refreshAll(
                followerPosition,
                followerVelocity,
                followerAppliedVolts,
                followerSupplyCurrent,
                followerTorqueCurrent,
                followerTemperature,
            )

        inputs.leaderConnected = leaderStatus.isOK
        inputs.followerConnected = followerStatus.isOK

        inputs.leaderPositionMeters = leaderPosition.valueAsDouble
        inputs.leaderVelocityMetersPerSec = leaderVelocity.valueAsDouble
        inputs.leaderAppliedVolts = leaderAppliedVolts.valueAsDouble
        inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.valueAsDouble
        inputs.leaderTorqueCurrentAmps = leaderTorqueCurrent.valueAsDouble
        inputs.leaderTempFahrenheit = (leaderTemperature.valueAsDouble * (9 / 5)) + 32

        inputs.followerPositionMeters = followerPosition.valueAsDouble
        inputs.followerVelocityMetersPerSec = followerVelocity.valueAsDouble
        inputs.followerAppliedVolts = followerAppliedVolts.valueAsDouble
        inputs.followerSupplyCurrentAmps = followerSupplyCurrent.valueAsDouble
        inputs.followerTorqueCurrentAmps = followerTorqueCurrent.valueAsDouble
        inputs.followerTempFahrenheit = (followerTemperature.valueAsDouble * (9 / 5)) + 32

        inputs.closedLoopPositionReference = closedLoopPositionReference.valueAsDouble
        inputs.closedLoopVelocityReference = closedLoopVelocityReference.valueAsDouble
        inputs.closedLoopOutput = closedLoopOutput.valueAsDouble
    }

    override fun setControl(control: ControlRequest) {
        motor.setControl(control)
    }

    override fun setConfig(config: TalonFXConfiguration, tries: Int, timeout: Double) {
        PhoenixUtil.tryUntilOk(tries) { motor.configurator.apply(config, timeout) }
        PhoenixUtil.tryUntilOk(tries) { follower.configurator.apply(config, timeout) }
    }

    override fun setBrake(enable: Boolean) {
        motor.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
        follower.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }
}
