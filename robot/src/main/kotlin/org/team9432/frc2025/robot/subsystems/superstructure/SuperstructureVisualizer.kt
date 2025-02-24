package org.team9432.frc2025.robot.subsystems.superstructure

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import org.team9432.frc2025.robot.subsystems.superstructure.arm.ArmConstants

class SuperstructureVisualizer(private val path: String) {
    fun publish(elevatorMeters: Double, armRotations: Double) {
        Logger.recordOutput("$path/A_Stage2", Pose3d(0.0, 0.0, elevatorMeters, Rotation3d.kZero))
        Logger.recordOutput(
            "$path/B_CoralArm",
            Pose3d(
                Units.inchesToMeters(-8.25),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(19.157754) + elevatorMeters,
                Rotation3d(0.0, Units.rotationsToRadians(armRotations - ArmConstants.MIN_POSITION), 0.0),
            ),
        )
    }
}
