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
        val coralAlignedVelocityThreshold = LoggedTunableNumber("Rollers/CoralCollectedThresholdRPS", 5.0)
        val coralAlignedDebounceTime = LoggedTunableNumber("Rollers/CoralAlignedDebounce", 1.0)

        val algaeCollectionThresholdRPS = LoggedTunableNumber("Rollers/AlgaeCollectionThresholdRPS", 5.0)
        val algaeCollectionDebounceTime = LoggedTunableNumber("Rollers/AlgaeCollectionDebounce", 1.0)

        val algaeDroppedThresholdRPS = LoggedTunableNumber("Rollers/AlgaeCollectionThresholdRPS", 10.0)
        val algaeDroppedDebounceTime = LoggedTunableNumber("Rollers/AlgaeCollectionDebounce", 1.2)
    }

    enum class State {
        IDLE,
        INTAKE_CORAL,
        SCORE_CORAL,
        UNJAM_CORAL,
        INTAKE_ALGAE,
        HOLD_ALGAE,
        SCORE_ALGAE,
    }

    var state: State = State.IDLE
        private set

    private var coralAlignedDebouncer = Debouncer(coralAlignedDebounceTime.get())
    private var algaeCollectedDebouncer = Debouncer(algaeCollectionDebounceTime.get())
    private var algaeDroppedDebouncer = Debouncer(algaeDroppedDebounceTime.get())

    init {
        defaultCommand = runGoal(State.IDLE)

        LoggedTunableNumber.ifChanged(hashCode(), coralAlignedDebounceTime) { (dt) ->
            coralAlignedDebouncer = Debouncer(dt)
        }
        LoggedTunableNumber.ifChanged(hashCode(), algaeCollectionDebounceTime) { (dt) ->
            algaeCollectedDebouncer = Debouncer(dt)
        }
        LoggedTunableNumber.ifChanged(hashCode(), algaeDroppedDebounceTime) { (dt) ->
            algaeDroppedDebouncer = Debouncer(dt)
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

            State.UNJAM_CORAL -> {
                funnel.goal = Funnel.Goal.UNJAM_CORAL
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

    fun runGoal(state: State) = startEnd({ this.state = state }, { this.state = State.IDLE })

    val coralCollected = Trigger {
        state == State.INTAKE_CORAL &&
            coralAlignedDebouncer.calculate(abs(manipulator.velocityRPS) < coralAlignedVelocityThreshold.get())
    }

    val algaeCollected = Trigger {
        state == State.INTAKE_ALGAE &&
            algaeCollectedDebouncer.calculate(abs(manipulator.velocityRPS) < algaeCollectionThresholdRPS.get())
    }
    val algaeDropped = Trigger {
        false
        //        state == State.HOLD_ALGAE &&
        //            algaeDroppedDebouncer.calculate(abs(manipulator.velocityRPS) >
        // algaeDroppedThresholdRPS.get())
    }
}
