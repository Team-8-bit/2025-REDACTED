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
        UP,
        DOWN,
        CLIMB,
    }

    init {
        defaultCommand = runGoal(Goal.IDLE)
    }

    private var goal = Goal.IDLE

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climber", inputs)

        when (goal) {
            Goal.IDLE -> io.setControl(neutralOut)
            Goal.UP -> io.setControl(voltageControl.withOutput(10.0))
            Goal.DOWN -> io.setControl(voltageControl.withOutput(-10.0))
            Goal.CLIMB -> io.setControl(torqueCurrent.withOutput(40.0))
        }
    }

    fun runGoal(goal: Goal) = run { this.goal = goal }
}
