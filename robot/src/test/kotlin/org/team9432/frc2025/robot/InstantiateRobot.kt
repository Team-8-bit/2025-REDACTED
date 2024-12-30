package org.team9432.frc2025.robot

import kotlin.test.Test

internal class InstantiateRobot {
    @Test
    fun compRobot() {
        // Set constants for the comp robot
        Constants.overrideUnitTestRobotType(Constants.RobotType.COMP)
        val robot = Robot()

        // Init robot
        robot.robotInit()

        // Start with the robot disabled
        robot.disabledInit()
        robot.disabledPeriodic()
        robot.robotPeriodic()
        robot.disabledExit()

        // Enable auto
        robot.autonomousInit()
        robot.autonomousPeriodic()
        robot.robotPeriodic()
        robot.autonomousExit()

        // Disable
        robot.disabledInit()
        robot.disabledPeriodic()
        robot.robotPeriodic()
        robot.disabledExit()

        // Enable teleop
        robot.teleopInit()
        robot.teleopPeriodic()
        robot.robotPeriodic()
        robot.teleopExit()

        // Disable
        robot.disabledInit()
        robot.disabledPeriodic()
        robot.robotPeriodic()
        robot.disabledExit()

        // Enable test
        robot.testInit()
        robot.testPeriodic()
        robot.robotPeriodic()
        robot.testExit()

        // Disable
        robot.disabledInit()
        robot.disabledPeriodic()
        robot.robotPeriodic()
    }

    @Test
    fun simRobot() {
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
