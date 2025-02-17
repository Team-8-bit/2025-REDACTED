package org.team9432.frc2025.robot.subsystems.coralrollers.dispenser

import com.ctre.phoenix6.hardware.TalonFX
import org.team9432.frc2025.robot.RobotMap

class Dispenser() {
    enum class Goal(private val setpointSupplier: () -> Double) {
        IDLE({ 0.0 }),
        INTAKE({ 5.0 });

        val volts get() = setpointSupplier.invoke()
    }

    var goal = Goal.IDLE

    private val motor = TalonFX(RobotMap.coralArmDispenser.canID, RobotMap.coralArmDispenser.canBus)

    fun periodic() {
        motor.setVoltage(goal.volts)
    }
}
