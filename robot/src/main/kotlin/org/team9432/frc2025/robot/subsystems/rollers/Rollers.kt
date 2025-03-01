package org.team9432.frc2025.robot.subsystems.rollers

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.subsystems.rollers.dispenser.Manipulator
import org.team9432.frc2025.robot.subsystems.rollers.funnel.Funnel

class Rollers(private val funnel: Funnel, private val manipulator: Manipulator) : SubsystemBase() {
    companion object {
        val coralAlignedVelocityThreshold = LoggedTunableNumber("Rollers/CoralAlignedVelocityThresholdRPS", 5.0)
        val coralAlignedDebounceTime = LoggedTunableNumber("Rollers/CoralAlignedDebounceTime", 1.0)
    }

    enum class State {
        IDLE,
        INTAKE_CORAL,
        SCORE_CORAL,
        INTAKE_ALGAE,
        HOLD_ALGAE,
        SCORE_ALGAE,
    }

    var state: State = State.IDLE
        private set

    private var coralAlignedDebouncer = Debouncer(coralAlignedDebounceTime.get())

    init {
        defaultCommand = runGoal(State.IDLE)

        LoggedTunableNumber.ifChanged(hashCode(), coralAlignedDebounceTime) { (dt) ->
            coralAlignedDebouncer = Debouncer(dt)
        }
    }

    override fun periodic() {
        funnel.periodic()
        manipulator.periodic()

        funnel.goal = Funnel.Goal.IDLE
        manipulator.goal = Manipulator.Goal.IDLE

        when (state) {
            State.IDLE -> {}
            State.INTAKE_CORAL -> {
                funnel.goal = Funnel.Goal.INTAKE_CORAL
                manipulator.goal = Manipulator.Goal.INTAKE_CORAL
            }

            State.SCORE_CORAL -> {
                manipulator.goal = Manipulator.Goal.OUTTAKE_CORAL
            }

            State.INTAKE_ALGAE -> {
                manipulator.goal = Manipulator.Goal.INTAKE_ALGAE
            }

            State.HOLD_ALGAE -> {
                manipulator.goal = Manipulator.Goal.INTAKE_ALGAE
            }

            State.SCORE_ALGAE -> {
                manipulator.goal = Manipulator.Goal.SCORE_ALGAE
            }
        }

        Logger.recordOutput("Rollers/State", state)
    }

    fun runGoal(state: State) = run { this.state = state }

    val coralCollected = Trigger {
        state == State.INTAKE_CORAL &&
            coralAlignedDebouncer.calculate(abs(manipulator.velocityRPS) < coralAlignedVelocityThreshold.get())
    }
}
