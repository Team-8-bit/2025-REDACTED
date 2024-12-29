package org.team9432.frc2025.lib.util

import com.ctre.phoenix6.StatusCode

object PhoenixUtil {
    /** Invokes and prints the result of [lazyMessage] if the status code is not [StatusCode.OK]. Returns itself. */
    fun StatusCode.printOnError(lazyMessage: (StatusCode) -> String): StatusCode {
        if (!this.isOK) println(lazyMessage.invoke(this))
        return this
    }

    /** Attempts to run the command until no error is produced.  */
    fun tryUntilOk(maxAttempts: Int, command: () -> StatusCode) {
        for (i in 0 until maxAttempts) {
            val error = command.invoke()
            if (error.isOK) break
        }
    }
}