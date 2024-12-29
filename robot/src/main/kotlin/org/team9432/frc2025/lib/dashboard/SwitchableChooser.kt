package org.team9432.frc2025.lib.dashboard

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import org.littletonrobotics.junction.networktables.LoggedNetworkString
import org.team9432.frc2025.lib.RobotPeriodicManager

private const val PLACEHOLDER: String = "<NA>"

// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/util/SwitchableChooser.java
class SwitchableChooser(tablePath: String, name: String) {
    private var options = arrayOf(PLACEHOLDER)
    private var active: String? = PLACEHOLDER

    private val table: NetworkTable = NetworkTableInstance.getDefault().getTable(tablePath).getSubTable(name)
    private val namePublisher = table.getStringTopic(".name").publish()
    private val typePublisher = table.getStringTopic(".type").publish()
    private val optionsPublisher = table.getStringArrayTopic("options").publish()
    private val defaultPublisher = table.getStringTopic("default").publish()
    private val activePublisher = table.getStringTopic("active").publish()
    private val selectedInput = LoggedNetworkString("$tablePath/$name/selected")

    init {
        namePublisher.set(name)
        typePublisher.set("String Chooser")
        optionsPublisher.set(this.options)
        defaultPublisher.set(this.options.first())
        activePublisher.set(this.options.first())
        selectedInput.set(this.options.first())

        RobotPeriodicManager.startPeriodic { periodic() }
    }

    /** Updates the set of available options. */
    fun setOptions(options: Array<String>) {
        if (options.contentEquals(this.options)) return
        this.options = if (options.isNotEmpty()) options else arrayOf(PLACEHOLDER)
        optionsPublisher.set(this.options)
        periodic()
    }

    /** Returns the selected option. */
    fun get(): String? = if (active == PLACEHOLDER) null else active

    private fun periodic() {
        val selected = selectedInput.get()

        active = options.firstOrNull { it != PLACEHOLDER && it == selected }

        if (active == null) {
            active = options.first()
            selectedInput.set(active)
        }

        defaultPublisher.set(active)
        activePublisher.set(active)
    }
}
