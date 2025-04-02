package org.team9432.frc2025.robot.subsystems.rollers

import edu.wpi.first.wpilibj.DigitalInput
import org.team9432.annotation.Logged
import org.team9432.frc2025.robot.RobotMap

class RollerSensors {
    private val frontCoralSensor = DigitalInput(RobotMap.FRONT_CORAL_LINEBREAK)

    fun updateInputs(inputs: RollerSensorsInputs) {
        inputs.frontCoralTripped = !frontCoralSensor.get()
    }

    @Logged
    open class RollerSensorsInputs {
        var frontCoralTripped: Boolean = false
    }
}
