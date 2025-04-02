package org.team9432.frc2025.robot.util

import edu.wpi.first.wpilibj2.command.Commands
import org.team9432.frc2025.lib.dashboard.AutoSelector
import org.team9432.frc2025.robot.Auto
import org.team9432.frc2025.robot.Localizer
import org.team9432.frc2025.robot.commands.drive.DrivetrainSimpleFeedforward
import org.team9432.frc2025.robot.commands.drive.WheelRadiusCharacterization
import org.team9432.frc2025.robot.subsystems.drive.Drive
import org.team9432.frc2025.robot.subsystems.superstructure.Superstructure
import org.team9432.frc2025.robot.util.FieldConstants.Reef.Branch

class AutoChooser(
    private val autoCommands: Auto,
    private val localizer: Localizer,
    private val drive: Drive,
    private val superstructure: Superstructure,
) {
    var command = Commands.none()
        private set

    private val autoChoosers =
        List(5) { AutoSelector.DashboardQuestion("Option $it Chooser", "Option $it Question") }.toSet()

    private val chooser =
        AutoSelector(autoChoosers) {
                addQuestion("Which Auto?", { command = it }) {
                    addOption("Do Nothing (Broken)", Commands::none)

                    addOption("Max L4 Left", { autoCommands.maxL4Left() })
                    addOption("Max L4 Right", { autoCommands.maxL4Right() })

                    addOption("Only L2") {
                        addQuestion("Side", { command = it }) {
                            addOption("Left", { autoCommands.onlyL2(Branch.J) })
                            addOption("Right", { autoCommands.onlyL2(Branch.E) })
                            var branch: Branch? = null
                            addOption("Custom", { branch?.let { autoCommands.onlyL2(it) } }) {
                                addQuestion("Which Branch?", { branch = it }) {
                                    Branch.entries.forEach { addOption(it.name, { it }) }
                                }
                            }
                        }
                    }
                    addOption("Only L4") {
                        addQuestion("Side", { command = it }) {
                            addOption("Left", { autoCommands.onlyL4(Branch.J) })
                            addOption("Right", { autoCommands.onlyL4(Branch.E) })
                            var branch: Branch? = null
                            addOption("Custom", { branch?.let { autoCommands.onlyL4(it) } }) {
                                addQuestion("Which Branch?", { branch = it }) {
                                    Branch.entries.forEach { addOption(it.name, { it }) }
                                }
                            }
                        }
                    }

                    var characterizationAuto = Commands.none()
                    addOption("Characterization", { characterizationAuto }) {
                        addQuestion("Which routine?", { characterizationAuto = it }) {
                            addOption(
                                "Drive Simple Feedforward Characterization",
                                { DrivetrainSimpleFeedforward(drive) },
                            )
                            addOption(
                                "Drive Wheel Radius Characterization",
                                { WheelRadiusCharacterization(drive, localizer) },
                            )
                            addOption(
                                "Elevator Static Characterization",
                                { superstructure.elevatorStaticCharacterization() },
                            )
                            addOption(
                                "CoralArm Static Characterization",
                                { superstructure.armStaticCharacterization() },
                            )
                        }
                    }
                }
            }
            .also { it.update() }

    fun update() {
        chooser.update()
    }
}
