package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.units.measure.*
import org.team9432.frc2025.lib.util.PhoenixUtil
import org.team9432.frc2025.robot.RobotMap

open class ElevatorIOReal : ElevatorIO {
    protected val talon = TalonFX(RobotMap.leftElevator.canID, RobotMap.leftElevator.canBus)
    protected val follower = TalonFX(RobotMap.rightElevator.canID, RobotMap.rightElevator.canBus)

    private val position: StatusSignal<Angle> = talon.position
    private val velocity: StatusSignal<AngularVelocity> = talon.velocity
    private val closedLoopPositionReference: StatusSignal<Double> = talon.closedLoopReference
    private val closedLoopVelocityReference: StatusSignal<Double> = talon.closedLoopReferenceSlope
    private val closedLoopOutput: StatusSignal<Double> = talon.closedLoopOutput

    private val leaderAppliedVolts: StatusSignal<Voltage> = talon.motorVoltage
    private val leaderSupplyCurrent: StatusSignal<Current> = talon.supplyCurrent
    private val leaderTorqueCurrent: StatusSignal<Current> = talon.torqueCurrent
    private val leaderTemperature: StatusSignal<Temperature> = talon.deviceTemp

    private val followerAppliedVolts: StatusSignal<Voltage> = follower.motorVoltage
    private val followerSupplyCurrent: StatusSignal<Current> = follower.supplyCurrent
    private val followerTorqueCurrent: StatusSignal<Current> = follower.torqueCurrent
    private val followerTemperature: StatusSignal<Temperature> = follower.deviceTemp

    private val config =
        TalonFXConfiguration().apply {
            Slot0 = Slot0Configs()
            Slot1 = Slot1Configs()

            Slot0.GravityType = GravityTypeValue.Elevator_Static
            Slot1.GravityType = GravityTypeValue.Elevator_Static
            Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign
            Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign

            MotorOutput.Inverted = InvertedValue.Clockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.PEAK_TORQUE_AMPS
            TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimit = ElevatorConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimitEnable = true

            Feedback.SensorToMechanismRatio = ElevatorConstants.MOTOR_ROTATIONS_PER_METER
        }

    init {
        PhoenixUtil.tryUntilOk(5) { talon.configurator.apply(config) }
        PhoenixUtil.tryUntilOk(5) { follower.configurator.apply(config) }

        follower.setControl(Follower(RobotMap.leftElevator.canID, /* OpposeMasterDirection= */ true))

        BaseStatusSignal.setUpdateFrequencyForAll(200.0, position, velocity)

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            closedLoopPositionReference,
            closedLoopVelocityReference,
            closedLoopOutput,
            leaderAppliedVolts,
            leaderSupplyCurrent,
            leaderTorqueCurrent,
            leaderTemperature,
            followerAppliedVolts,
            followerSupplyCurrent,
            followerTorqueCurrent,
            followerTemperature,
        )

        talon.optimizeBusUtilization()
        follower.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
        val leaderStatus =
            BaseStatusSignal.refreshAll(
                position,
                velocity,
                closedLoopPositionReference,
                closedLoopVelocityReference,
                closedLoopOutput,
                leaderAppliedVolts,
                leaderSupplyCurrent,
                leaderTorqueCurrent,
                leaderTemperature,
            )
        val followerStatus =
            BaseStatusSignal.refreshAll(
                followerAppliedVolts,
                followerSupplyCurrent,
                followerTorqueCurrent,
                followerTemperature,
            )

        inputs.leaderConnected = leaderStatus.isOK
        inputs.followerConnected = followerStatus.isOK

        inputs.positionMeters = position.valueAsDouble
        inputs.velocityMetersPerSec = velocity.valueAsDouble
        inputs.closedLoopPositionReference = closedLoopPositionReference.valueAsDouble
        inputs.closedLoopVelocityReference = closedLoopVelocityReference.valueAsDouble
        inputs.closedLoopOutput = closedLoopOutput.valueAsDouble

        inputs.leaderAppliedVolts = leaderAppliedVolts.valueAsDouble
        inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.valueAsDouble
        inputs.leaderTorqueCurrentAmps = leaderTorqueCurrent.valueAsDouble
        inputs.leaderTempFahrenheit = (leaderTemperature.valueAsDouble * (9 / 5)) + 32

        inputs.followerAppliedVolts = followerAppliedVolts.valueAsDouble
        inputs.followerSupplyCurrentAmps = followerSupplyCurrent.valueAsDouble
        inputs.followerTorqueCurrentAmps = followerTorqueCurrent.valueAsDouble
        inputs.followerTempFahrenheit = (followerTemperature.valueAsDouble * (9 / 5)) + 32
    }

    override fun setControl(control: ControlRequest) {
        talon.setControl(control)
    }

    override fun updateConfig(block: TalonFXConfiguration.() -> Unit) {
        block.invoke(config)
        PhoenixUtil.tryUntilOk(5) { talon.configurator.apply(config) }
    }

    override fun setSensorPosition(meters: Double) {
        talon.setPosition(meters)
    }

    override fun setBrake(enable: Boolean) {
        Thread {
                talon.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
                follower.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
            }
            .start()
    }
}
