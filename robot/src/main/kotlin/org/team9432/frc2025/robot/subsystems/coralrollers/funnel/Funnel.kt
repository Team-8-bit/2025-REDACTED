package org.team9432.frc2025.robot.subsystems.coralrollers.funnel

import com.ctre.phoenix6.hardware.TalonFX
import org.team9432.frc2025.robot.RobotMap

class Funnel() {
    enum class Goal(private val setpointSupplier: () -> Double) {
        IDLE({ 0.0 }),
        INTAKE({ 5.0 });

        val volts get() = setpointSupplier.invoke()
    }

    var goal = Goal.IDLE

    private val motor = TalonFX(RobotMap.funnel.canID, RobotMap.funnel.canBus)

    fun periodic() {
        motor.setVoltage(goal.volts)
    }
}
