package org.team9432.frc2025.lib

object RobotPeriodicManager {
    private val periodics = mutableSetOf<() -> Any>()

    // Have an intermediate list to prevent modifying the main one while it's being iterated through
    private val periodicsToStart = mutableSetOf<() -> Any>()

    fun startPeriodic(function: () -> Any) {
        periodicsToStart.add(function)
    }

    fun invokeAll() {
        // Add all newly scheduled periodics
        periodics.addAll(periodicsToStart)
        periodicsToStart.clear()
        // Invoke periodics
        periodics.forEach { it.invoke() }
    }
}
