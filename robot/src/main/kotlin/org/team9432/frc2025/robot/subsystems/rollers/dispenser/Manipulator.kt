package org.team9432.frc2025.robot.subsystems.rollers.dispenser

import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import org.littletonrobotics.junction.Logger

class Manipulator(private val io: ManipulatorIO) {
    private val inputs = LoggedManipulatorIOInputs()

    private val neutralOut = NeutralOut()
    private val currentControl = TorqueCurrentFOC(0.0)
    private val voltageControl = VoltageOut(0.0)

    enum class Goal {
        IDLE,
        INTAKE_CORAL,
        OUTTAKE_CORAL,
        INTAKE_ALGAE,
        HOLD_ALGAE,
        SCORE_ALGAE,
    }

    var goal = Goal.IDLE

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Manipulator", inputs)

        when (goal) {
            Goal.IDLE -> io.setControl(neutralOut)
            Goal.INTAKE_CORAL -> io.setControl(voltageControl.withOutput(5.0))
            Goal.OUTTAKE_CORAL -> io.setControl(voltageControl.withOutput(-6.0))
            Goal.INTAKE_ALGAE -> io.setControl(voltageControl.withOutput(-10.0))
            Goal.HOLD_ALGAE -> io.setControl(currentControl.withOutput(-30.0))
            Goal.SCORE_ALGAE -> io.setControl(voltageControl.withOutput(12.0))
        }

        Logger.recordOutput("Rollers/ManipulatorState", goal)
    }

    val velocityRPS
        get() = inputs.velocityRotationsPerSec

    val torqueCurrentAmps
        get() = inputs.torqueCurrentAmps
}
