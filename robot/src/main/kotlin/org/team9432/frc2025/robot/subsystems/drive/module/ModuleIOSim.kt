package org.team9432.frc2025.robot.subsystems.drive.module

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.team9432.frc2025.robot.subsystems.drive.module.ModuleIO.ModuleIOInputs

class ModuleIOSim(private val moduleSim: SwerveModuleSimulation): ModuleIO {
    private val driveMotor = moduleSim
        .useGenericMotorControllerForDrive()
        .withCurrentLimit(Amps.of(60.0))

    private val steerMotor = moduleSim
        .useGenericControllerForSteer()
        .withCurrentLimit(Amps.of(20.0))

    private val driveFeedback: PIDController = PIDController(1.0, 0.0, 0.0)
    private val steerFeedback: PIDController = PIDController(8.0, 0.0, 0.2)

    private var angleSetpoint: Rotation2d? = null
    private var speedSetpoint: Double? = null
    private var speedFeedforward: Double = 0.0

    init {
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI)
        driveFeedback.setTolerance(0.25)
    }

    /** Updates the inputs with the latest sensor information. */
    override fun updateInputs(inputs: ModuleIOInputs) {
        speedSetpoint?.let { targetSpeed ->
            val targetVolts = driveFeedback.calculate(inputs.driveVelocityRadPerSecond, targetSpeed) + speedFeedforward
            driveMotor.requestVoltage(Volts.of(targetVolts))
        }
        angleSetpoint?.let { targetAngle ->
            val targetVolts = steerFeedback.calculate(inputs.steerAbsolutePosition.radians, targetAngle.radians)
            steerMotor.requestVoltage(Volts.of(targetVolts))
        }

        inputs.driveConnected = true
        inputs.steerConnected = true
        inputs.cancoderConnected = true

        inputs.drivePositionRads = moduleSim.driveWheelFinalPosition.`in`(Radians)
        inputs.driveVelocityRadPerSecond = moduleSim.driveWheelFinalSpeed.`in`(RadiansPerSecond)
        inputs.driveAppliedVolts = moduleSim.driveMotorAppliedVoltage.`in`(Volts)
        inputs.driveSupplyCurrentAmps = moduleSim.driveMotorSupplyCurrent.`in`(Amps)

        inputs.steerAbsolutePosition = moduleSim.steerAbsoluteFacing
        inputs.steerPosition = Rotation2d.fromRotations(moduleSim.steerRelativeEncoderPosition.`in`(Rotations))
        inputs.steerVelocityRadPerSec = moduleSim.steerRelativeEncoderVelocity.`in`(RadiansPerSecond)
        inputs.steerAppliedVolts = moduleSim.driveMotorAppliedVoltage.`in`(Volts)
        inputs.steerSupplyCurrentAmps = moduleSim.steerMotorSupplyCurrent.`in`(Amps)

        inputs.odometryDrivePositionsRads = moduleSim.cachedDriveWheelFinalPositions.map { it.`in`(Radians) }.toDoubleArray()
        inputs.odometrySteerPositions = moduleSim.cachedSteerAbsolutePositions
    }

    /** Runs the drive motor at the specified voltage. */
    override fun runDriveVoltage(volts: Double) {
        speedSetpoint = null
        driveMotor.requestVoltage(Volts.of(volts))
    }

    /** Runs the steer motor at the specified voltage. */
    override fun runSteerVoltage(volts: Double) {
        angleSetpoint = null
        steerMotor.requestVoltage(Volts.of(volts))
    }

    /** Runs the drive motor at the specified velocity with the given feedforward. */
    override fun runDriveVelocity(velocityRadPerSec: Double, feedforward: Double) {
        speedSetpoint = velocityRadPerSec
    }

    /** Runs the steer motor to the specified position. */
    override fun runSteerPosition(angle: Rotation2d) {
        angleSetpoint = angle
    }

    /** Sets the pid constants of the drive motor. */
    override fun setDrivePID(p: Double, i: Double, d: Double) = driveFeedback.setPID(p, i, d)

    /** Sets the pid constants of the steer motor. */
    override fun setSteerPID(p: Double, i: Double, d: Double) = steerFeedback.setPID(p, i, d)
}