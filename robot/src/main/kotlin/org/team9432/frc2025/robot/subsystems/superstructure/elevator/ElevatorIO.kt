package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import org.team9432.annotation.Logged

interface ElevatorIO {
    @Logged
    open class ElevatorIOInputs {
        var leaderConnected: Boolean = true
        var leaderPositionMeters: Double = 0.0
        var leaderVelocityMetersPerSec: Double = 0.0
        var leaderAppliedVolts: Double = 0.0
        var leaderSupplyCurrentAmps: Double = 0.0
        var leaderTorqueCurrentAmps: Double = 0.0
        var leaderTempFahrenheit: Double = 0.0
        var closedLoopReference: Double = 0.0

        var followerConnected: Boolean = true
        var followerPositionMeters: Double = 0.0
        var followerVelocityMetersPerSec: Double = 0.0
        var followerAppliedVolts: Double = 0.0
        var followerSupplyCurrentAmps: Double = 0.0
        var followerTorqueCurrentAmps: Double = 0.0
        var followerTempFahrenheit: Double = 0.0
    }

    /** Updates the inputs with the latest sensor information. */
    fun updateInputs(inputs: ElevatorIOInputs) {}

    /** Runs the motors at the specified voltage. */
    fun runVoltage(volts: Double) {}

    /** Runs the motors at the specified current. */
    fun runAmps(amps: Double) {}

    /** Runs the elevator to the specified position with the given feedforward. */
    fun runPosition(positionMeters: Double, feedforward: Double) {}

    /** Sets the pid constants of the motors. */
    fun setPID(p: Double, i: Double, d: Double) {}

    /** Sets the feedforward constants of the motors. */
    fun setFF(s: Double, g: Double, v: Double, a: Double) {}

    /** Sets the motion magic constants of the motors. */
    fun setMotionMagic(cruise: Double, accel: Double, jerk: Double) {}

    /** Enables or disables brake mode on the motors. */
    fun setBrake(enable: Boolean) {}

    /** Runs the motors at neutral output. */
    fun stop() {}
}
