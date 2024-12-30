package org.team9432.frc2025.lib.dashboard

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.util.*

/** Class for selecting different auto configurations. */
class AutoSelector(private val choosers: Set<DashboardQuestion>, buildQuestions: AutoSelectorQuestionScope.() -> Unit) {
    private val baseQuestions = AutoSelectorQuestionScope().apply(buildQuestions)

    /** Updates this selector's questions based on the current state of the dashboard chooser. */
    fun update() {
        // Get new values for each chooser
        choosers.forEach { it.chooser.update() }

        // Create a queue of all the available choosers
        val chooserQueue: Queue<DashboardQuestion> = LinkedList(choosers)

        // Update the choosers with questions from the auto selector
        baseQuestions.update(chooserQueue)

        // Blank out any that weren't set
        chooserQueue.forEach { it.clear() }
    }

    class AutoSelectorQuestionScope internal constructor() {
        private val questions = mutableListOf<AutoSelectorQuestion<*>>()

        /** Add a new question */
        fun <T> addQuestion(
            question: String,
            onSelect: ((T) -> Unit)? = null,
            addOptions: AutoSelectorOptionScope<T>.() -> Unit,
        ) {
            questions.add(AutoSelectorQuestion(AutoSelectorOptionScope<T>(question).apply(addOptions), onSelect))
        }

        // Returns true if something changed
        internal fun update(choosers: Queue<DashboardQuestion>): Boolean {
            // This can't be shortened to questions.any {...} because it will stop after the first
            // changed one
            return questions.map { it.update(choosers) }.any { it }
        }

        // This class is needed to retain the type of the question so it can be passed to onSelect()
        private data class AutoSelectorQuestion<T>(
            private val options: AutoSelectorOptionScope<T>,
            private val onSelect: ((T) -> Unit)?,
        ) {
            // Returns true if something changed
            fun update(choosers: Queue<DashboardQuestion>): Boolean {
                val (valueUpdated, newValue) = options.update(choosers)
                if (valueUpdated && newValue != null) {
                    onSelect?.invoke(newValue.invoke())
                }
                return valueUpdated
            }
        }
    }

    class AutoSelectorOptionScope<T> internal constructor(private val question: String) {
        private val options = mutableMapOf<String, Pair<(() -> T)?, AutoSelectorQuestionScope?>>()

        /** Add an option to the question, with optional nested questions to show if this option is selected. */
        fun addOption(
            answer: String,
            value: (() -> T)? = null,
            buildQuestions: (AutoSelectorQuestionScope.() -> Unit)? = null,
        ) {
            options[answer] = value to buildQuestions?.let { AutoSelectorQuestionScope().apply(it) }
        }

        private var lastAnswer: String? = null



















        // Returns a boolean signifying if something changed and the selected value
        internal fun update(choosers: Queue<DashboardQuestion>): Pair<Boolean, (() -> T)?> {
            // Take a chooser from the list
            // This also removes the chooser so that the same one is never set twice in one loop
            val targetChooser = choosers.poll()

            // Display the question and options
            targetChooser.set(question, options.keys)

            val answer = targetChooser.chooser.get()

            // Pass the remaining choosers to any nested questions
            val nestedChange = options[answer]?.second?.update(choosers)

            if (answer != lastAnswer || nestedChange == true) {
                lastAnswer = answer
                return true to options[answer]?.first
            } else {
                return false to options[answer]?.first
            }
        }
    }

    data class DashboardQuestion(private val chooserKey: String, private val questionKey: String) {
        // The leading '/' before "AutoSelector" is needed for this to work:
        // https://discord.com/channels/176186766946992128/528555967827148801/1322768035198795836
        // (read up for the problem)
        internal val chooser = SwitchableChooser("/AutoSelector", chooserKey)

        internal fun clear() = set("", emptySet())

        internal fun set(question: String, options: Collection<String>) {
            SmartDashboard.putString("AutoSelector/$questionKey", question)
            chooser.setOptions(options.toTypedArray())
        }
    }
}
