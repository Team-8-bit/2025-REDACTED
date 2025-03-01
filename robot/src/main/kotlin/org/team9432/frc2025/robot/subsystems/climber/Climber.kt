package org.team9432.frc2025.robot.subsystems.climber

import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Climber(private val io: ClimberIO) : SubsystemBase() {
    private val inputs = LoggedClimberIOInputs()

    private val neutralOut = NeutralOut()
    private val voltageControl = VoltageOut(0.0).withEnableFOC(true)
    private val torqueCurrent = TorqueCurrentFOC(0.0)

    enum class Goal {
        IDLE,
        PREPARE,
        UP,
        DOWN,
        HOLD,
    }

    private var goal = Goal.IDLE

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climber", inputs)

        when (goal) {
            Goal.IDLE -> io.setControl(neutralOut)
            Goal.PREPARE -> io.setControl(voltageControl.withOutput(6.0))
            Goal.UP -> io.setControl(torqueCurrent.withOutput(120.0))
            Goal.DOWN -> io.setControl(voltageControl.withOutput(-6.0))
            Goal.HOLD -> io.setControl(torqueCurrent.withOutput(5.0))
        }
    }

    fun runGoal(goal: Climber.Goal) = startEnd({ this.goal = goal }, { this.goal = Goal.IDLE })
}
