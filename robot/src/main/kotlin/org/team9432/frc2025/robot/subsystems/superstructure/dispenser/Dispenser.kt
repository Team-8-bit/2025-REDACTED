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
        INTAKE_ALGAE,
        HOLD_ALGAE,
        SCORE_ALGAE_PROCESSOR,
        SCORE_ALGAE_NET,
    }

    var goal = Goal.IDLE

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Dispenser", inputs)

        when (goal) {
            Goal.IDLE -> io.setControl(neutralOut)
            Goal.INTAKE_CORAL -> io.setControl(voltageControl.withOutput(5.0))
            Goal.OUTTAKE_CORAL -> io.setControl(voltageControl.withOutput(-6.0))
            Goal.INTAKE_ALGAE -> io.setControl(voltageControl.withOutput(-10.0))
            Goal.HOLD_ALGAE -> io.setControl(currentControl.withOutput(-40.0))
            Goal.SCORE_ALGAE_PROCESSOR -> io.setControl(voltageControl.withOutput(5.0))
            Goal.SCORE_ALGAE_NET -> io.setControl(voltageControl.withOutput(12.0))
        }
    }
}
