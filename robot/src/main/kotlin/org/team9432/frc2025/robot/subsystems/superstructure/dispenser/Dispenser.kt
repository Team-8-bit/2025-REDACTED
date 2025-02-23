package org.team9432.frc2025.robot.subsystems.superstructure.dispenser

import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import org.littletonrobotics.junction.Logger

class Dispenser(private val io: DispenserIO) {
    private val inputs = LoggedDispenserIOInputs()

    private val neutralOut = NeutralOut()
    private val currentControl = TorqueCurrentFOC(0.0)
    private val voltageControl = VoltageOut(0.0)

    enum class Goal {
        IDLE,
        INTAKE_CORAL,
        OUTTAKE_CORAL,
    }

    var goal = Goal.IDLE

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Dispenser", inputs)

        when (goal) {
            Goal.IDLE -> {
                io.setControl(neutralOut)
            }
            Goal.INTAKE_CORAL -> {
                io.setControl(voltageControl.withOutput(5.0))
            }
            Goal.OUTTAKE_CORAL -> {
                io.setControl(voltageControl.withOutput(-6.0))
            }
        }
    }
}
