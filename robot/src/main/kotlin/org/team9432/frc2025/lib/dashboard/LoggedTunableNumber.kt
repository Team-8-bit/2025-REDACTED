// By 6328
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/a025615a52193b7709db7cf14c51c57be17826f2/src/main/java/org/littletonrobotics/frc2024/subsystems/drive/Drive.java
package org.team9432.frc2025.lib.dashboard

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import kotlin.reflect.KProperty

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or value not in
 * dashboard.
 */
class LoggedTunableNumber(key: String, private val defaultValue: Double): () -> Double {
    private var dashboardNumber: LoggedNetworkNumber? = null
    private val lastHasChangedValues: MutableMap<Int, Double> = HashMap()

    init {
        if (isTuningMode) {
            dashboardNumber = LoggedNetworkNumber("$TABLE_KEY/$key", defaultValue)
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    fun get() = if (isTuningMode) dashboardNumber?.get() ?: defaultValue else defaultValue

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects. Recommended
     *   approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false otherwise.
     */
    fun hasChanged(id: Int): Boolean {
        val currentValue = get()
        val lastValue = lastHasChangedValues[id]
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues[id] = currentValue
            return true
        }
        return false
    }

    override fun invoke() = get()

    operator fun getValue(thisRef: Any?, property: KProperty<*>) = get()

    companion object {
        private var isTuningMode = false

        /**
         * Sets if tuning mode should be enabled. In tuning mode all [LoggedTunableNumber]s will be displayed on the
         * dashboard and robot code will listen to changes. If tuning mode is disabled the code will use the default
         * values provided (recommended for competitions).
         */
        fun setTuningModeEnabled(enabled: Boolean) {
            isTuningMode = enabled
        }

        fun isTuningModeEnabled(): Boolean {
            return isTuningMode
        }

        private const val TABLE_KEY = "TunableNumbers"

        /**
         * Runs action if any of the tunableNumbers have changed
         *
         * @param id Unique identifier for the caller to avoid conflicts when shared between multiple * objects.
         *   Recommended approach is to pass the result of "hashCode()"
         * @param action Callback to run when any of the tunable numbers have changed. Access tunable numbers in order
         *   inputted in method
         * @param tunableNumbers All tunable numbers to check
         */
        fun ifChanged(id: Int, vararg tunableNumbers: LoggedTunableNumber, action: (List<Double>) -> Unit) {
            if (!isTuningMode) return
            if (tunableNumbers.any { it.hasChanged(id) }) {
                action.invoke(tunableNumbers.map { it.get() })
            }
        }
    }
}
