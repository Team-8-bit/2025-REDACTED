package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.*

class ElevatorIOKraken : ElevatorIO {
    private val leader = TalonFX(ElevatorConstants.LEADER_ID, "*")
    private val follower = TalonFX(ElevatorConstants.FOLLOWER_ID, "*")

    private val leaderPosition: StatusSignal<Angle> = leader.position
    private val leaderVelocity: StatusSignal<AngularVelocity> = leader.velocity
    private val leaderAppliedVolts: StatusSignal<Voltage> = leader.motorVoltage
    private val leaderSupplyCurrent: StatusSignal<Current> = leader.supplyCurrent
    private val leaderTorqueCurrent: StatusSignal<Current> = leader.torqueCurrent
    private val leaderTemperature: StatusSignal<Temperature> = leader.deviceTemp

    private val followerPosition: StatusSignal<Angle> = leader.position
    private val followerVelocity: StatusSignal<AngularVelocity> = follower.velocity
    private val followerAppliedVolts: StatusSignal<Voltage> = follower.motorVoltage
    private val followerSupplyCurrent: StatusSignal<Current> = follower.supplyCurrent
    private val followerTorqueCurrent: StatusSignal<Current> = follower.torqueCurrent
    private val followerTemperature: StatusSignal<Temperature> = follower.deviceTemp

    private val voltageControl = VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0)
    private val currentControl = TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val motionMagicPositionControl = MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)

    private val leaderConfig =
        TalonFXConfiguration().apply {
            Slot0.kP = ElevatorConstants.gains.kP
            Slot0.kI = ElevatorConstants.gains.kI
            Slot0.kD = ElevatorConstants.gains.kD

            MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.PEAK_TORQUE_AMPS
            TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimit = ElevatorConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimitEnable = true

            Feedback.SensorToMechanismRatio = ElevatorConstants.REDUCTION
        }

    init {
        follower.setControl(Follower(ElevatorConstants.LEADER_ID, /* OpposeMasterDirection= */ true))

        leader.configurator.apply(leaderConfig, 1.0)

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
        )

        leader.optimizeBusUtilization(0.0, 1.0)
        follower.optimizeBusUtilization(0.0, 1.0)
    }

    override fun updateInputs(inputs: ElevatorIO.ElevatorIOInputs) {
        val leaderStatus =
            BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderAppliedVolts,
                leaderSupplyCurrent,
                leaderTorqueCurrent,
                leaderTemperature,
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
    }

    /** Runs the motors at the specified voltage. */
    override fun runVoltage(volts: Double) {
        leader.setControl(voltageControl.withOutput(volts))
    }

    /** Runs the motors at the specified current. */
    override fun runAmps(amps: Double) {
        leader.setControl(currentControl.withOutput(amps))
    }

    /** Runs the elevator to the specified position with the given feedforward. */
    override fun runPosition(positionMeters: Double, feedforward: Double) {
        leader.setControl(motionMagicPositionControl.withPosition(positionMeters).withFeedForward(feedforward))
    }

    /** Sets the pid constants of the motors. */
    override fun setPID(p: Double, i: Double, d: Double) {
        leaderConfig.Slot0.kP = p
        leaderConfig.Slot0.kI = i
        leaderConfig.Slot0.kD = d
        leader.configurator.apply(leaderConfig, 0.01)
    }

    /** Sets the motion magic constants of the motors. */
    override fun setMotionMagic(jerk: Double, accel: Double, cruise: Double) {
        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = cruise
        leaderConfig.MotionMagic.MotionMagicAcceleration = accel
        leaderConfig.MotionMagic.MotionMagicJerk = jerk
        leader.configurator.apply(leaderConfig, 0.01)
    }

    /** Enables or disables brake mode on the motors. */
    override fun setBrake(enable: Boolean) {
        leader.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
        follower.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }
}
