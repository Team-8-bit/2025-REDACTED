package org.team9432.frc2025.robot.subsystems.superstructure.arm

import com.ctre.phoenix6.configs.TalonFXConfiguration
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class TunableArmGains(
    ntPath: String,
    kP: Double,
    kD: Double,
    kS: Double,
    kG: Double,
    velocity: Double,
    acceleration: Double,
    jerk: Double,
) {
    private val kP = LoggedTunableNumber("$ntPath/kP", kP)
    private val kD = LoggedTunableNumber("$ntPath/kD", kD)
    private val kS = LoggedTunableNumber("$ntPath/kS", kS)
    private val kG = LoggedTunableNumber("$ntPath/kG", kG)

    private val velocity = LoggedTunableNumber("$ntPath/Velocity", velocity)
    private val acceleration = LoggedTunableNumber("$ntPath/Acceleration", acceleration)
    private val jerk = LoggedTunableNumber("$ntPath/Jerk", jerk)

    fun ifChanged(id: Int, function: () -> Unit) {
        val hasChanged = LoggedTunableNumber.hasChanged(id, kP, kD, kS, kG, velocity, acceleration, jerk)

        if (hasChanged) {
            function.invoke()
        }
    }

    fun applyToTalonFXConfig(config: TalonFXConfiguration) {
        config.apply {
            Slot0.withKP(kP.get()).withKD(kD.get())
            Slot0.withKS(kS.get()).withKG(kG.get())
            MotionMagic.withMotionMagicCruiseVelocity(velocity.get())
                .withMotionMagicAcceleration(acceleration.get())
                .withMotionMagicJerk(jerk.get())
        }
    }
}
