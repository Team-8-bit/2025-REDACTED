package org.team9432.frc2025.robot.subsystems.rollers.funnel

import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class Funnel(private val io: FunnelIO) {
    private val inputs = LoggedFunnelIOInputs()

    private val neutralOut = NeutralOut()
    private val voltageControl = VoltageOut(0.0)

    enum class Goal(private val voltageSupplier: () -> Double) {
        IDLE({ 0.0 }),
        INTAKE_CORAL(LoggedTunableNumber("Funnel/Setpoints/IntakeCoral", 8.0)),
        UNJAM_CORAL(LoggedTunableNumber("Funnel/Setpoints/UnjamCoral", -12.0)),
        INTAKE_UNJAM_COMBO({ 0.0 }),
        EJECT_ALGAE(LoggedTunableNumber("Funnel/Setpoints/EjectAlgae", -12.0));

        val voltage
            get() = voltageSupplier.invoke()
    }

    val unjamTimer = Timer()

    var goal = Goal.IDLE

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Funnel", inputs)

        var voltage = 0.0

        if (goal == Goal.IDLE) {
            io.setControl(neutralOut)
        } else if (goal == Goal.INTAKE_UNJAM_COMBO) {
            // Works, potential durability issues
            //            val intakeTime = unjamComboIntakeTime.get()
            //            val unjamTime = unjamComboUnjamTime.get()
            //
            //            if (!unjamTimer.isRunning || unjamTimer.hasElapsed(intakeTime +
            // unjamTime)) {
            //                unjamTimer.restart()
            //            }
            //
            //            if (!unjamTimer.hasElapsed(intakeTime)) {
            //                io.setControl(voltageControl.withOutput(Goal.INTAKE_CORAL.voltage))
            //                voltage = voltageControl.Output
            //            } else {
            //                io.setControl(voltageControl.withOutput(Goal.UNJAM_CORAL.voltage))
            //                voltage = voltageControl.Output
            //            }
            io.setControl(voltageControl.withOutput(Goal.INTAKE_CORAL.voltage))
        } else {
            io.setControl(voltageControl.withOutput(goal.voltage))
            voltage = voltageControl.Output
        }

        Logger.recordOutput("Rollers/Voltage", voltage)
        Logger.recordOutput("Rollers/FunnelState", goal)
    }

    companion object {
        val unjamComboIntakeTime = LoggedTunableNumber("Rollers/unjamComboIntakeTime", 1.0)
        val unjamComboUnjamTime = LoggedTunableNumber("Rollers/unjamComboUnjamTime", 0.1)
    }
}
