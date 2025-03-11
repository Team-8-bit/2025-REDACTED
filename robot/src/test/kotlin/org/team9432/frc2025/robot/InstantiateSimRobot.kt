package org.team9432.frc2025.robot

import kotlin.test.Test

internal class InstantiateSimRobot {
    @Test
    fun instantiateSimRobot() {
        // Set constants for the sim robot
        Constants.overrideUnitTestRobotType(Constants.RobotType.SIM)
        val robot = Robot()

        // Init robot
        robot.robotInit()
        robot.simulationInit()

        // Start with the robot disabled
        robot.disabledInit()
        robot.disabledPeriodic()
        robot.robotPeriodic()
        robot.simulationPeriodic()
        robot.disabledExit()

        // Enable auto
        robot.autonomousInit()
        robot.autonomousPeriodic()
        robot.robotPeriodic()
        robot.simulationPeriodic()
        robot.autonomousExit()

        // Disable
        robot.disabledInit()
        robot.disabledPeriodic()
        robot.robotPeriodic()
        robot.simulationPeriodic()
        robot.disabledExit()

        // Enable teleop
        robot.teleopInit()
        robot.teleopPeriodic()
        robot.robotPeriodic()
        robot.simulationPeriodic()
        robot.teleopExit()

        // Disable
        robot.disabledInit()
        robot.disabledPeriodic()
        robot.robotPeriodic()
        robot.simulationPeriodic()
        robot.disabledExit()

        // Enable test
        robot.testInit()
        robot.testPeriodic()
        robot.robotPeriodic()
        robot.simulationPeriodic()
        robot.testExit()

        // Disable
        robot.disabledInit()
        robot.disabledPeriodic()
        robot.robotPeriodic()
        robot.simulationPeriodic()
    }
}
