package org.team9432.frc2025.robot.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import edu.wpi.first.wpilibj.RobotController
import java.util.*
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock
import kotlin.math.roundToLong
import org.team9432.annotation.Logged

object OdometryThread : Thread() {
    /** Prevent concurrent modification of the queue lists. */
    private val signalsLock: Lock = ReentrantLock()

    /** Signals registered and tracked by the thread. */
    private var signals: Array<BaseStatusSignal> = emptyArray()

    /** Queues with the values of each signal. */
    private val queues = mutableListOf<Queue<Double>>()

    /** Queue with the timestamps of the values in the signal queues. */
    private val timestampQueue: Queue<Double> = ArrayBlockingQueue(20)

    init {
        name = "OdometryThread"
        isDaemon = true
    }

    fun updateInputs(inputs: OdometryThreadInputs) {
        inputs.timestamps = timestampQueue.toDoubleArray()
        timestampQueue.clear()
    }

    /** Registers a signal to run in the odometry thread and returns a queue that is filled with the received values. */
    fun registerSignal(signal: StatusSignal<*>): Queue<Double> {
        val queue = ArrayBlockingQueue<Double>(20)
        signalsLock.lock()
        Drive.odometryLock.lock()
        try {
            // Add the signal and queue to their respective lists
            signals += signal
            queues.add(queue)
        } finally {
            signalsLock.unlock()
            Drive.odometryLock.unlock()
        }

        return queue
    }

    override fun run() {
        while (true) {
            // Wait for signal updates
            signalsLock.withLock {
                if (signals.isNotEmpty()) {
                    BaseStatusSignal.waitForAll(2.0 / DrivetrainConstants.ODOMETRY_FREQUENCY, *signals)
                } else {
                    // Simulation
                    sleep((1000.0 / DrivetrainConstants.ODOMETRY_FREQUENCY).roundToLong())
                }
            }

            // Save data
            Drive.odometryLock.withLock {
                // Add each new value to its respective queue
                for (i in signals.indices) {
                    queues[i].offer(signals[i].valueAsDouble)
                }

                var timestamp = RobotController.getFPGATime() / 1e6

                // Subtract average latency from the timestamp
                var totalLatency = 0.0
                for (signal in signals) {
                    totalLatency += signal.timestamp.latency
                }

                if (signals.isNotEmpty()) {
                    timestamp -= totalLatency / signals.size
                }

                // Add the timestamp to the queue
                timestampQueue.offer(timestamp)
            }
        }
    }

    @Logged
    open class OdometryThreadInputs {
        var timestamps: DoubleArray = doubleArrayOf()
    }
}
