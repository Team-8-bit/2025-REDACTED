package org.team9432.frc2025.robot.commands.drive

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import java.text.DecimalFormat
import java.text.NumberFormat
import org.team9432.frc2025.robot.subsystems.drive.Drive

fun DrivetrainSimpleFeedforward(drive: Drive): Command {
    val steerSetpoints = Array(4) { Rotation2d() }
    val velocitySamples: MutableList<Double> = mutableListOf()
    val amperageSamples: MutableList<Double> = mutableListOf()
    val timer: Timer = Timer()

    return Commands.sequence( // Reset data
            Commands.runOnce({
                velocitySamples.clear()
                amperageSamples.clear()
            }), // Allow modules to orient
            Commands.run({ drive.runDriveCharacterizationAmperage({ 0.0 }, steerSetpoints) }, drive)
                .withTimeout(2.0), // Start timer
            Commands.runOnce(timer::restart), // Accelerate and gather data
            drive.runDriveCharacterizationAmperage(
                {
                    val amps: Double = timer.get() * 1.0 // a/s
                    velocitySamples.add(drive.getModuleCharacterizationVelocityRotationsPerSecond().average())
                    amperageSamples.add(amps)
                    amps
                },
                steerSetpoints,
            ),
        ) // When cancelled, calculate and print results
        .finallyDo { interrupted ->
            val n = velocitySamples.size
            var sumX = 0.0
            var sumY = 0.0
            var sumXY = 0.0
            var sumX2 = 0.0
            for (i in 0 until n) {
                sumX += velocitySamples[i]
                sumY += amperageSamples[i]
                sumXY += velocitySamples[i] * amperageSamples[i]
                sumX2 += velocitySamples[i] * velocitySamples[i]
            }
            val kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX)
            val kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX)

            val formatter: NumberFormat = DecimalFormat("#0.00000")
            println("********** Drive FF Characterization Results **********")
            println("\tkS: " + formatter.format(kS))
            println("\tkV: " + formatter.format(kV))
        }
}
