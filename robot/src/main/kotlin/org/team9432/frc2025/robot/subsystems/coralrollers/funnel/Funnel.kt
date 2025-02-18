package org.team9432.frc2025.robot.subsystems.coralrollers.funnel

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import org.team9432.frc2025.robot.RobotMap

class Funnel() {
    enum class Goal(private val setpointSupplier: () -> Double) {
        IDLE({ 0.0 }),
        INTAKE({ 5.0 });

        val volts
            get() = setpointSupplier.invoke()
    }

    var goal = Goal.IDLE

    private val motor = TalonFX(RobotMap.funnel.canID, RobotMap.funnel.canBus)

    init {
        motor.configurator.apply(
            TalonFXConfiguration().withMotorOutput(MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
        )
    }

    fun periodic() {
        motor.setVoltage(goal.volts)
    }
}
