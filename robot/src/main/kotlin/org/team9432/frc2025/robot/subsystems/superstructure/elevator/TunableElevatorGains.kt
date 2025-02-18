package org.team9432.frc2025.robot.subsystems.superstructure.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class TunableElevatorGains(
    ntPath: String,
    kP: Double,
    kD: Double,
    kSStage1: Double,
    kGStage1: Double,
    kSStage2: Double,
    kGStage2: Double,
    velocity: Double,
    acceleration: Double,
    jerk: Double,
) {
    private val kP = LoggedTunableNumber("$ntPath/kP", kP)
    private val kD = LoggedTunableNumber("$ntPath/kD", kD)
    private val kSStage1 = LoggedTunableNumber("$ntPath/kSStage1", kSStage1)
    private val kGStage1 = LoggedTunableNumber("$ntPath/kGStage1", kGStage1)
    private val kSStage2 = LoggedTunableNumber("$ntPath/kSStage2", kSStage2)
    private val kGStage2 = LoggedTunableNumber("$ntPath/kGStage2", kGStage2)

    private val velocity = LoggedTunableNumber("$ntPath/Velocity", velocity)
    private val acceleration = LoggedTunableNumber("$ntPath/Acceleration", acceleration)
    private val jerk = LoggedTunableNumber("$ntPath/Jerk", jerk)

    fun ifChanged(id: Int, function: () -> Unit) {
        val hasChanged =
            LoggedTunableNumber.hasChanged(
                id,
                kP,
                kD,
                kSStage1,
                kGStage1,
                kSStage2,
                kGStage2,
                velocity,
                acceleration,
                jerk,
            )

        if (hasChanged) {
            function.invoke()
        }
    }

    fun applyToTalonFXConfig(config: TalonFXConfiguration) {
        config.apply {
            Slot0.withKP(kP.get()).withKD(kD.get())
            Slot1.withKP(kP.get()).withKD(kD.get())
            Slot0.withKS(kSStage1.get()).withKG(kGStage1.get())
            Slot1.withKS(kSStage2.get()).withKG(kGStage2.get())
            MotionMagic.withMotionMagicCruiseVelocity(velocity.get())
                .withMotionMagicAcceleration(acceleration.get())
                .withMotionMagicJerk(jerk.get())
        }
    }
}
