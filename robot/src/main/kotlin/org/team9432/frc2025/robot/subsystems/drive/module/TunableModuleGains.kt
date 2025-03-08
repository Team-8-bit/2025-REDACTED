package org.team9432.frc2025.robot.subsystems.drive.module

import com.ctre.phoenix6.configs.TalonFXConfiguration
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class TunableModuleGains(
    ntPath: String,
    kPDrive: Double,
    kDDrive: Double,
    kSDrive: Double,
    kVDrive: Double,
    kPSteer: Double,
    kDSteer: Double,
    mmCruiseSteer: Double,
    mmAccelSteer: Double,
) {
    private val kPDrive = LoggedTunableNumber("$ntPath/kPDrive", kPDrive)
    private val kDDrive = LoggedTunableNumber("$ntPath/kDDrive", kDDrive)
    private val kSDrive = LoggedTunableNumber("$ntPath/kSDrive", kSDrive)
    private val kVDrive = LoggedTunableNumber("$ntPath/kVDrive", kVDrive)
    private val kPSteer = LoggedTunableNumber("$ntPath/kPSteer", kPSteer)
    private val kDSteer = LoggedTunableNumber("$ntPath/kDSteer", kDSteer)
    private val mmCruiseSteer = LoggedTunableNumber("$ntPath/mmCruiseSteer", mmCruiseSteer)
    private val mmAccelSteer = LoggedTunableNumber("$ntPath/mmAccelSteer", mmAccelSteer)

    fun ifDriveChanged(id: Int, function: () -> Unit) {
        val hasChanged = LoggedTunableNumber.hasChanged(id, kPDrive, kDDrive, kSDrive, kVDrive)

        if (hasChanged) {
            function.invoke()
        }
    }

    fun ifSteerChanged(id: Int, function: () -> Unit) {
        val hasChanged = LoggedTunableNumber.hasChanged(id, kPSteer, kDSteer, mmCruiseSteer, mmAccelSteer)

        if (hasChanged) {
            function.invoke()
        }
    }

    fun applyToDriveConfig(config: TalonFXConfiguration) {
        config.apply {
            Slot0.withKP(kPDrive.get()).withKD(kDDrive.get())
            Slot0.withKS(kSDrive.get()).withKG(kVDrive.get())
        }
    }

    fun applyToSteerConfig(config: TalonFXConfiguration) {
        config.apply {
            Slot0.withKP(kPSteer.get()).withKD(kDSteer.get())
            MotionMagic.withMotionMagicCruiseVelocity(mmCruiseSteer.get())
                .withMotionMagicAcceleration(mmAccelSteer.get())
        }
    }
}
