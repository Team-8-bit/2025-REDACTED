package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.wpilibj.Alert
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class Elevator(private val io: ElevatorIO) {
    private val inputs: LoggedElevatorIOInputs = LoggedElevatorIOInputs()

    private val leaderDisconnectedAlert = Alert("Leader (left) elevator motor disconnected!", Alert.AlertType.kError)
    private val followerDisconnectedAlert = Alert("Follower (right) elevator motor disconnected!", Alert.AlertType.kError)

    private val kP = LoggedTunableNumber("Elevator/kP", ElevatorConstants.gains.kP)
    private val kI = LoggedTunableNumber("Elevator/kI", ElevatorConstants.gains.kI)
    private val kD = LoggedTunableNumber("Elevator/kD", ElevatorConstants.gains.kD)
    private val ffkS = LoggedTunableNumber("Elevator/ffkS", ElevatorConstants.gains.ffkS)
    private val ffkV = LoggedTunableNumber("Elevator/ffkV", ElevatorConstants.gains.ffkV)
    private val ffkA = LoggedTunableNumber("Elevator/ffkA", ElevatorConstants.gains.ffkA)
    private val ffkG = LoggedTunableNumber("Elevator/ffkG", ElevatorConstants.gains.ffkG)

    private var feedforward = ElevatorFeedforward(ffkS.get(), ffkG.get(), ffkV.get(), ffkA.get(), 0.0)

    init {
        io.setBrake(true)
    }

    fun updateInputs() {
        io.updateInputs(inputs)
        Logger.processInputs("Elevator", inputs)

        leaderDisconnectedAlert.set(!inputs.leaderConnected)
        followerDisconnectedAlert.set(!inputs.followerConnected)
    }
}
