package org.team9432.frc2025.robot.subsystems.rollers.funnel

import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VoltageOut
import org.littletonrobotics.junction.Logger

class Funnel(private val io: FunnelIO) {
    private val inputs = LoggedFunnelIOInputs()

    private val neutralOut = NeutralOut()
    private val voltageControl = VoltageOut(0.0)

    enum class Goal(private val voltageSupplier: () -> Double) {
        IDLE({ 0.0 }),
        INTAKE_CORAL({ 4.0 }),
        EJECT_ALGAE({ -12.0 });

        val voltage
            get() = voltageSupplier.invoke()
    }

    var goal = Goal.IDLE

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Funnel", inputs)

        if (goal == Goal.IDLE) {
            io.setControl(neutralOut)
        } else {
            io.setControl(voltageControl.withOutput(goal.voltage))
        }
    }
}
