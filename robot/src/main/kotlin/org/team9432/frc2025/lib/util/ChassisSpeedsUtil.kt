package org.team9432.frc2025.lib.util

import edu.wpi.first.math.kinematics.ChassisSpeeds
import kotlin.math.abs
import kotlin.math.hypot

fun ChassisSpeeds.velocityLessThan(metersPerSecond: Double, radiansPerSecond: Double) =
    hypot(vxMetersPerSecond, vyMetersPerSecond) < metersPerSecond && abs(omegaRadiansPerSecond) < radiansPerSecond