package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.*
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.units.measure.*
import org.team9432.frc2025.robot.RobotMap

class ElevatorIOKraken : ElevatorIO {
    private val leader = TalonFX(RobotMap.leftElevator.canID, RobotMap.leftElevator.canBus)
    private val follower = TalonFX(RobotMap.rightElevator.canID, RobotMap.rightElevator.canBus)

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

    private val closedLoopReference = leader.closedLoopReference

    private val voltageControl = VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0)
    private val currentControl = TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val motionMagicPositionControl = PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0)
    private val neutralOut = NeutralOut()

    private val leaderConfig =
        TalonFXConfiguration().apply {
            Slot0.kP = ElevatorConstants.gains.kP
            Slot0.kI = ElevatorConstants.gains.kI
            Slot0.kD = ElevatorConstants.gains.kD

            Slot0.kS = ElevatorConstants.gains.ffkS
            Slot0.kV = ElevatorConstants.gains.ffkV
            Slot0.kA = ElevatorConstants.gains.ffkA
            Slot0.kG = ElevatorConstants.gains.ffkG

            Slot0.GravityType = GravityTypeValue.Elevator_Static
            Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign

            MotorOutput.Inverted = InvertedValue.Clockwise_Positive
            MotorOutput.NeutralMode = NeutralModeValue.Brake

            TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.PEAK_TORQUE_AMPS
            TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimit = ElevatorConstants.PEAK_TORQUE_AMPS
            CurrentLimits.StatorCurrentLimitEnable = true

            Feedback.SensorToMechanismRatio = ElevatorConstants.REDUCTION
        }

    init {
        follower.setControl(Follower(RobotMap.leftElevator.canID, /* OpposeMasterDirection= */ true))

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
            closedLoopReference,
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
        inputs.closedLoopReference = closedLoopReference.valueAsDouble

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
        leader.setControl(motionMagicPositionControl.withPosition(positionMeters)) // .withFeedForward(feedforward))
    }

    /** Sets the pid constants of the motors. */
    override fun setPID(p: Double, i: Double, d: Double) {
        leaderConfig.Slot0.kP = p
        leaderConfig.Slot0.kI = i
        leaderConfig.Slot0.kD = d
        leader.configurator.apply(leaderConfig, 0.1)
    }

    /** Sets the feedforward constants of the motors. */
    override fun setFF(s: Double, g: Double, v: Double, a: Double) {
        leaderConfig.Slot0.kS = s
        leaderConfig.Slot0.kG = g
        leaderConfig.Slot0.kV = v
        leaderConfig.Slot0.kA = a
        leader.configurator.apply(leaderConfig, 0.1)
    }

    /** Sets the motion magic constants of the motors. */
    override fun setMotionMagic(cruise: Double, accel: Double, jerk: Double) {
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

    /** Runs the motors at neutral output. */
    override fun stop() {
        leader.setControl(neutralOut)
    }
}
