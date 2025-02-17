package org.team9432.frc2025.robot.subsystems.superstructure.coralarm

import com.revrobotics.sim.SparkMaxSim
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import kotlin.math.hypot
import org.littletonrobotics.junction.LoggedRobot

class CoralArmIOSim : CoralArmIONeo() {
    private val armSim =
        SingleJointedArmSim(
            /* gearbox = */ DCMotor.getNEO(1),
            /* gearing = */ CoralArmConstants.REDUCTION,
            /* jKgMetersSquared = */ 0.04,
            /* armLengthMeters = */ Units.inchesToMeters(hypot(4.48, 3.98)),
            /* minAngleRads = */ Units.rotationsToRadians(0.0),
            /* maxAngleRads = */ Units.rotationsToRadians(0.5),
            /* simulateGravity = */ true,
            /* startingAngleRads = */ Units.rotationsToRadians(0.0),
        )

    private val motorSim = SparkMaxSim(motor, DCMotor.getNEO(1))

    override fun updateInputs(inputs: CoralArmIO.CoralArmIOInputs) {
        armSim.setInputVoltage(motorSim.appliedOutput * RoboRioSim.getVInVoltage())
        armSim.update(LoggedRobot.defaultPeriodSecs)

        motorSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(armSim.velocityRadPerSec),
            RoboRioSim.getVInVoltage(),
            LoggedRobot.defaultPeriodSecs,
        )

        super.updateInputs(inputs)
    }
}
