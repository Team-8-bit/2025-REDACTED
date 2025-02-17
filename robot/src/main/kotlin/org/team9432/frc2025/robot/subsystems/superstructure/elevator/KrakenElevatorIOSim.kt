package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.sim.ChassisReference
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import org.littletonrobotics.junction.LoggedRobot

class KrakenElevatorIOSim : KrakenElevatorIOReal() {
    private val elevatorSim =
        ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            ElevatorConstants.REDUCTION,
            Units.lbsToKilograms(25.0),
            ElevatorConstants.DRUM_RADIUS,
            ElevatorConstants.MIN_POSITION,
            ElevatorConstants.MAX_POSITION,
            /* simulateGravity = */ true,
            /* startingHeightMeters = */ 0.0,
        )

    private val leaderSim: TalonFXSimState = super.motor.simState
    private val followerSim: TalonFXSimState = super.follower.simState

    init {
        leaderSim.Orientation = ChassisReference.Clockwise_Positive
        followerSim.Orientation = ChassisReference.Clockwise_Positive
    }

    override fun updateInputs(inputs: KrakenElevatorIO.ElevatorIOInputs) {
        leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        followerSim.setSupplyVoltage(RobotController.getBatteryVoltage())

        elevatorSim.setInputVoltage(leaderSim.motorVoltage)
        elevatorSim.update(LoggedRobot.defaultPeriodSecs)

        leaderSim.setRawRotorPosition(elevatorSim.positionMeters * ElevatorConstants.MOTOR_ROTATIONS_PER_METER)
        followerSim.setRawRotorPosition(elevatorSim.positionMeters * ElevatorConstants.MOTOR_ROTATIONS_PER_METER)

        leaderSim.setRotorVelocity(elevatorSim.velocityMetersPerSecond * ElevatorConstants.MOTOR_ROTATIONS_PER_METER)
        followerSim.setRotorVelocity(elevatorSim.velocityMetersPerSecond * ElevatorConstants.MOTOR_ROTATIONS_PER_METER)

        super.updateInputs(inputs)
    }
}
