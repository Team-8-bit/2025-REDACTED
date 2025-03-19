package org.team9432.frc2025.robot.subsystems.rollers

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.abs
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber
import org.team9432.frc2025.robot.Constants
import org.team9432.frc2025.robot.RobotState
import org.team9432.frc2025.robot.subsystems.rollers.dispenser.Manipulator
import org.team9432.frc2025.robot.subsystems.rollers.funnel.Funnel

class Rollers(private val funnel: Funnel, private val manipulator: Manipulator) : SubsystemBase() {
    companion object {
        val coralAlignedTorqueCurrentThreshold =
            LoggedTunableNumber("Rollers/CoralCollectedThresholdTorqueCurrent", 13.0)
        val coralAlignedDebounceTime = LoggedTunableNumber("Rollers/CoralCollectedDebounce", 0.2)

        val algaeCollectionThresholdRPS = LoggedTunableNumber("Rollers/AlgaeCollectionThresholdRPS", 30.0)
        val algaeCollectionDebounceTime = LoggedTunableNumber("Rollers/AlgaeCollectionDebounce", 0.1)

        val algaeDroppedThresholdRPS = LoggedTunableNumber("Rollers/AlgaeDroppedThresholdRPS", 10.0)
        val algaeDroppedDebounceTime = LoggedTunableNumber("Rollers/AlgaeDroppedDebounce", 0.5)
    }

    enum class State {
        IDLE,
        INTAKE_CORAL,
        SCORE_CORAL_TALL,
        SCORE_CORAL_LOW,
        SCORE_CORAL_L1,
        UNJAM_CORAL,
        INTAKE_ALGAE,
        SCORE_ALGAE,
    }

    var state: State = State.IDLE
        private set

    var hasCoral = false
        private set

    var hasAlgae = false
        private set

    private var hasRunRemoveCoral = false
    var readyToRemoveCoral = false
        set(value) {
            if (hasRunRemoveCoral && value) {
                field = value
            }
        }

    val hasCoralTrigger = Trigger { hasCoral }
    val hasAlgaeTrigger = Trigger { hasAlgae }

    var disableBeambreak = { false }

    private var coralAlignedDebouncer = Debouncer(coralAlignedDebounceTime.get())
    private var algaeCollectedDebouncer = Debouncer(algaeCollectionDebounceTime.get())
    private var algaeDroppedDebouncer = Debouncer(algaeDroppedDebounceTime.get())

    private val rollerSensors = RollerSensors()
    private val rollerSensorInputs = LoggedRollerSensorsInputs()

    init {
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
        rollerSensors.updateInputs(rollerSensorInputs)
        Logger.processInputs("RollersSensors", rollerSensorInputs)

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

            State.SCORE_CORAL_TALL -> {
                manipulator.goal = Manipulator.Goal.OUTTAKE_CORAL_TALL
                hasRunRemoveCoral = true
            }

            State.SCORE_CORAL_LOW -> {
                manipulator.goal = Manipulator.Goal.OUTTAKE_CORAL_LOW
                hasRunRemoveCoral = true
            }

            State.SCORE_CORAL_L1 -> {
                manipulator.goal = Manipulator.Goal.OUTTAKE_CORAL_L1
                hasRunRemoveCoral = true
            }

            State.UNJAM_CORAL -> {
                funnel.goal = Funnel.Goal.UNJAM_CORAL
            }

            State.INTAKE_ALGAE -> {
                if (!hasAlgae) {
                    manipulator.goal = Manipulator.Goal.INTAKE_ALGAE
                } else {
                    manipulator.goal = Manipulator.Goal.HOLD_ALGAE
                }
            }

            State.SCORE_ALGAE -> {
                manipulator.goal = Manipulator.Goal.SCORE_ALGAE
                hasAlgae = false
            }
        }

        // Check if the coral has been collected
        val coralCurrentDetector =
            coralAlignedDebouncer.calculate(
                state == State.INTAKE_CORAL &&
                    abs(manipulator.torqueCurrentAmps) > coralAlignedTorqueCurrentThreshold.get()
            )

        val coralBeambreakDetector =
            (rollerSensorInputs.frontCoralTripped && state == State.INTAKE_CORAL) && !disableBeambreak()

        if ((coralCurrentDetector || coralBeambreakDetector) && !Constants.robot.isSim) {
            hasCoral = true
        }

        val algaeCollected =
            algaeCollectedDebouncer.calculate(
                state == State.INTAKE_ALGAE && abs(manipulator.velocityRPS) < algaeCollectionThresholdRPS.get()
            )
        if (algaeCollected && !Constants.robot.isSim) {
            hasAlgae = true
        }

        val algaeDropped =
            algaeDroppedDebouncer.calculate(
                hasAlgae && state == State.INTAKE_ALGAE && abs(manipulator.velocityRPS) > algaeDroppedThresholdRPS.get()
            )
        if (algaeDropped && !Constants.robot.isSim) {
            hasAlgae = false
        }

        if (hasRunRemoveCoral && readyToRemoveCoral) {
            hasCoral = false
            readyToRemoveCoral = false
            hasRunRemoveCoral = false
        }

        Logger.recordOutput("Rollers/State", state)
        SmartDashboard.putBoolean("Rollers/HasAlgae", hasAlgae)
        SmartDashboard.putBoolean("Rollers/HasCoral", hasCoral)
    }

    fun clearCoral() {
        hasCoral = false
    }

    fun preloadCoral() = Commands.runOnce({ hasCoral = true })

    fun runGoal(state: State) = runGoal { state }

    fun runGoal(state: () -> State) = run { this.state = state() }

    fun getScoringStateForTarget(target: RobotState.CoralScoringTarget) =
        when (target) {
            in setOf(RobotState.CoralScoringTarget.L2, RobotState.CoralScoringTarget.L3) -> {
                State.SCORE_CORAL_LOW
            }

            RobotState.CoralScoringTarget.L1 -> {
                State.SCORE_CORAL_L1
            }

            else -> {
                State.SCORE_CORAL_TALL
            }
        }

    fun simSetHasAlgae(hasAlgae: Boolean) {
        if (Constants.robot.isSim) this.hasAlgae = hasAlgae
    }

    fun simSetHasCoral(hasCoral: Boolean) {
        if (Constants.robot.isSim) this.hasCoral = hasCoral
    }
}
