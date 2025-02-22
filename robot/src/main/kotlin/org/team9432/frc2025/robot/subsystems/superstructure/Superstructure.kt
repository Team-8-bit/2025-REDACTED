package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.subsystems.superstructure.algaearm.AlgaeArm
import org.team9432.frc2025.robot.subsystems.superstructure.climber.Climber
import org.team9432.frc2025.robot.subsystems.superstructure.coralarm.CoralArm
import org.team9432.frc2025.robot.subsystems.superstructure.coralarm.CoralArmConstants
import org.team9432.frc2025.robot.subsystems.superstructure.elevator.Elevator

class Superstructure(
    private val elevator: Elevator,
    private val coralArm: CoralArm,
    private val algaeArm: AlgaeArm,
    private val climber: Climber,
) : SubsystemBase() {
    private var goal = Goal.STOW

    enum class Goal {
        STOW,
        TEST_ELEVATOR,
        ELEVATOR_AMP_INPUT,
        TEST_CORAL_ARM,
    }

    init {
        defaultCommand = runGoal(Goal.STOW)
    }

    override fun periodic() {
        if (DriverStation.isDisabled()) {
            goal = Goal.STOW
        }

        elevator.goal = Elevator.Goal.STOW

        when (goal) {
            Goal.STOW -> {
                elevator.goal = Elevator.Goal.STOW
                coralArm.goal = CoralArm.Goal.STOW
            }

            Goal.TEST_ELEVATOR -> {
                elevator.goal = Elevator.Goal.TEST
                coralArm.goal = CoralArm.Goal.STOW
            }

            Goal.TEST_CORAL_ARM -> {
                elevator.goal = Elevator.Goal.STOW
                coralArm.goal = CoralArm.Goal.TEST
            }

            Goal.ELEVATOR_AMP_INPUT -> {
                elevator.goal = Elevator.Goal.AMP_INPUT
                coralArm.goal = CoralArm.Goal.STOW
            }
        }

        elevator.periodic()
        coralArm.periodic()

        Logger.recordOutput(
            "Superstructure/Poses/A_Stage2",
            Pose3d(0.0, 0.0, elevator.positionMeters, Rotation3d.kZero),
        )
        Logger.recordOutput(
            "Superstructure/Poses/B_CoralArm",
            Pose3d(
                Units.inchesToMeters(-8.25),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(19.157754 + elevator.positionMeters),
                Rotation3d(
                    0.0,
                    Units.rotationsToRadians(coralArm.positionRotations - CoralArmConstants.MIN_POSITION),
                    0.0,
                ),
            ),
        )

        println(CoralArmConstants.MAX_POSITION)

        Logger.recordOutput("Superstructure/Goal", goal)
    }

    fun runGoal(newGoal: Goal): Command = startEnd({ goal = newGoal }, { goal = Goal.STOW })

    fun runElevatorCharacterizationAmps(amps: Double) {
        elevator.runCharacterizationAmps(amps)
    }

    fun getElevatorCharacterizationVelocity(): Double {
        return elevator.velocityMps
    }

    fun endElevatorCharacterization() {
        elevator.endCharacterization()
    }
}
