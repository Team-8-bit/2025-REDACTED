package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import org.team9432.frc2025.lib.util.temperatureFahrenheit
import org.team9432.frc2025.robot.RobotMap

open class CoralArmIONeo : CoralArmIO {
    protected val motor = SparkMax(RobotMap.coralArmPivot.canID, SparkLowLevel.MotorType.kBrushless)
    private val encoder = motor.encoder
    private val config = SparkMaxConfig()

    private var isBrakeMode = false

    init {
        config.apply {
            voltageCompensation(11.0)
            smartCurrentLimit(40)
            inverted(true)
            idleMode(SparkBaseConfig.IdleMode.kBrake)
        }

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun runVoltage(volts: Double) {
        motor.setVoltage(volts)
    }

    override fun setBrakeMode(enabled: Boolean) {
        if (enabled != isBrakeMode) {
            isBrakeMode = enabled
            val idleMode = if (enabled) SparkBaseConfig.IdleMode.kBrake else SparkBaseConfig.IdleMode.kCoast
            config.idleMode(idleMode)
            motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        }
    }

    override fun updateInputs(inputs: CoralArmIO.CoralArmIOInputs) {
        inputs.positionRotations = encoder.position / CoralArmConstants.REDUCTION
        inputs.velocityRPM = encoder.velocity / CoralArmConstants.REDUCTION
        inputs.appliedVoltage = motor.appliedOutput * motor.busVoltage
        inputs.supplyCurrentAmps = motor.outputCurrent
        inputs.tempFahrenheit = motor.temperatureFahrenheit
    }
}
