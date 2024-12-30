package org.team9432.frc2025.robot

import edu.wpi.first.wpilibj.RobotBase
import kotlin.system.exitProcess

object Constants {
    /** The robot this code is running on. */
    private val ROBOT_TYPE = RobotType.SIM

    /** Overrides the type of the robot, ONLY TO BE USED IN UNIT TESTS. */
    private var overrideUnitTestRobotType: RobotType? = null

    /** The robot this code is running on. */
    val robot: RobotType
        get() = overrideUnitTestRobotType ?: ROBOT_TYPE

    enum class RobotType {
        COMP,
        SIM;

        val isComp
            get() = this == COMP

        val isSim
            get() = this == SIM
    }

    enum class Mode {
        REAL,
        SIM,
        REPLAY;

        val isReal
            get() = this == REAL

        val isSim
            get() = this == SIM

        val isReplay
            get() = this == REPLAY
    }

    val mode: Mode
        get() =
            when (robot) {
                RobotType.COMP ->
                    if (RobotBase.isReal() || overrideUnitTestRobotType != null) Mode.REAL
                    else Mode.REPLAY // If the competition robot is selected but the robot isn't
                // real, it is running replay
                RobotType.SIM -> Mode.SIM
            }

    /** Overrides the type of the robot, ONLY TO BE USED IN UNIT TESTS. */
    fun overrideUnitTestRobotType(robot: RobotType) {
        overrideUnitTestRobotType = robot
    }
}

/** Checks whether the correct robot is selected when deploying. From team 6328. */
fun main() {
    if (Constants.robot.isSim) {
        System.err.println("Cannot deploy sim robot! Please change the value in Constants.kt!")
        exitProcess(1)
    }
}
