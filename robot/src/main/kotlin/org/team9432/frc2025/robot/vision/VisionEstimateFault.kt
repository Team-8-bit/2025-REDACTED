package org.team9432.frc2025.robot.vision

@JvmRecord
data class VisionEstimateFault(
    val outOfBounds: Boolean,
    val singleTagTooAmbiguous: Boolean,
    val noTags: Boolean,
    val infeasibleZValue: Boolean,
    val infeasiblePitchValue: Boolean,
    val infeasibleRollValue: Boolean,
    val isDisabled: Boolean,
) {
    val anyFaults: Boolean
        get() =
            outOfBounds ||
                singleTagTooAmbiguous ||
                noTags ||
                infeasibleZValue ||
                infeasiblePitchValue ||
                infeasibleRollValue ||
                isDisabled
}
