package org.team9432.frc2025.lib.util

import com.revrobotics.spark.SparkBase

/** Returns the temperature if this device in fahrenheit. */
val SparkBase.temperatureFahrenheit
    get() = (motorTemperature * (9 / 5)) + 32
