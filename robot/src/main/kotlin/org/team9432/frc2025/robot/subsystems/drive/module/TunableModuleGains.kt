package org.team9432.frc2025.robot.subsystems.drive.module

import com.ctre.phoenix6.configs.TalonFXConfiguration
import org.team9432.frc2025.lib.dashboard.LoggedTunableNumber

class TunableModuleGains(
    ntPath: String,
    kPDrive: Double,
    kDDrive: Double,
    kPSteer: Double,
    kDSteer: Double,
    kSDrive: Double,
    kVDrive: Double,
) {
    private val kPDrive = LoggedTunableNumber("$ntPath/kPDrive", kPDrive)
    private val kDDrive = LoggedTunableNumber("$ntPath/kDDrive", kDDrive)
    private val kPSteer = LoggedTunableNumber("$ntPath/kPSteer", kPSteer)
    private val kDSteer = LoggedTunableNumber("$ntPath/kDSteer", kDSteer)
    private val kSDrive = LoggedTunableNumber("$ntPath/kSDrive", kSDrive)
    private val kVDrive = LoggedTunableNumber("$ntPath/kVDrive", kVDrive)

    fun ifChanged(id: Int, function: () -> Unit) {
        val hasChanged =
            LoggedTunableNumber.hasChanged(
                id,
                kPDrive,
                kDDrive,
                kPSteer,
                kDSteer,
                kSDrive,
                kVDrive,
            )

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
        }
    }
}
