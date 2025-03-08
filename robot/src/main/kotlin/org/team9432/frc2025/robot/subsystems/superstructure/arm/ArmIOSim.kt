package org.team9432.frc2025.robot.subsystems.superstructure.arm

import com.ctre.phoenix6.sim.ChassisReference
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import kotlin.math.hypot
import org.littletonrobotics.junction.LoggedRobot

class ArmIOSim : ArmIOReal() {
    private val armSim =
        SingleJointedArmSim(
            /* gearbox = */ DCMotor.getKrakenX60Foc(1),
            /* gearing = */ ArmConstants.REDUCTION,
            /* jKgMetersSquared = */ 163.67174 * 0.00029263965, // onshape ft-lbs (xx measurement) to m-kg
            /* armLengthMeters = */ Units.inchesToMeters(hypot(4.911385, 2.136103)),
            /* minAngleRads = */ Units.rotationsToRadians(ArmConstants.MIN_POSITION),
            /* maxAngleRads = */ Units.rotationsToRadians(ArmConstants.MAX_POSITION),
            /* simulateGravity = */ true,
            /* startingAngleRads = */ Units.rotationsToRadians(ArmConstants.MIN_POSITION),
        )

    private val talonSim: TalonFXSimState = super.talon.simState

    init {
        talonSim.Orientation = ChassisReference.Clockwise_Positive
    }

    override fun updateInputs(inputs: ArmIO.ArmIOInputs) {
        talonSim.setSupplyVoltage(RobotController.getBatteryVoltage())

        armSim.setInputVoltage(talonSim.motorVoltage)
        armSim.update(LoggedRobot.defaultPeriodSecs)

        talonSim.setRawRotorPosition(Units.radiansToRotations(armSim.angleRads) * ArmConstants.REDUCTION)
        talonSim.setRotorVelocity(Units.radiansToRotations(armSim.velocityRadPerSec) * ArmConstants.REDUCTION)

        super.updateInputs(inputs)
    }
}
