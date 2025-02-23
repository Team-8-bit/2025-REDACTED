// By 6328
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/a025615a52193b7709db7cf14c51c57be17826f2/src/main/java/org/littletonrobotics/frc2024/subsystems/drive/Drive.java
package org.team9432.frc2025.lib.dashboard

import edu.wpi.first.wpilibj.DriverStation
import kotlin.reflect.KProperty
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.team9432.frc2025.robot.Constants.TUNING_MODE

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or value not in
 * dashboard. Default value must be passed in the constructor or via [initDefault] for the class to work.
 */
class LoggedTunableNumber(private val key: String) : () -> Double {
    private var dashboardNumber: LoggedNetworkNumber? = null
    private val lastHasChangedValues: MutableMap<Int, Double> = HashMap()

    constructor(key: String, defaultValue: Double) : this(key) {
        initDefault(defaultValue)
    }

    private var default: Double? = null
    private var reportedNoDefault = false

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    fun initDefault(defaultValue: Double) {
        if (default == null) {
            default = defaultValue
            if (TUNING_MODE) {
                dashboardNumber = LoggedNetworkNumber("$TABLE_KEY/$key", defaultValue)
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    fun get(): Double {
        val currentDefault = default
        return if (currentDefault == null) {
            if (!reportedNoDefault) {
                DriverStation.reportError("No default set for TunableNumber $key!", false)
                reportedNoDefault = true
            }
            0.0
        } else {
            if (TUNING_MODE) dashboardNumber?.get() ?: currentDefault else currentDefault
        }
    }

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
            if (!TUNING_MODE) return
            if (tunableNumbers.any { it.hasChanged(id) }) {
                action.invoke(tunableNumbers.map { it.get() })
            }
        }

        /**
         * Check if any of the given tunableNumbers have changed
         *
         * @param id Unique identifier for the caller to avoid conflicts when shared between multiple * objects.
         *   Recommended approach is to pass the result of "hashCode()"
         * @param tunableNumbers All tunable numbers to check
         */
        fun hasChanged(id: Int, vararg tunableNumbers: LoggedTunableNumber): Boolean {
            if (!TUNING_MODE) return false
            return tunableNumbers.any { it.hasChanged(id) }
        }
    }
}
