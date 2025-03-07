package org.team9432.frc2025.robot.vision

import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import java.nio.ByteBuffer

class AprilTagList(val tags: Collection<Int>) : StructSerializable {
    class AprilTagListStruct : Struct<AprilTagList> {
        private val tagCount = VisionConstants.aprilTagLayout.tags.size

        override fun getTypeClass() = AprilTagList::class.java

        override fun getSize() = Int.SIZE_BYTES

        override fun getTypeName() = "TrackedAprilTags"

        override fun getSchema() = List(tagCount) { tagNum -> "bool ID${tagNum + 1}:1;" }.joinToString(" ")

        override fun pack(bb: ByteBuffer, value: AprilTagList) {
            var packed = 0

            for (i in 1..tagCount) {
                if (value.tags.contains(i)) {
                    packed = packed or (1 shl i - 1)
                }
            }

            bb.putInt(packed)
        }

        override fun unpack(bb: ByteBuffer): AprilTagList {
            val value = bb.getInt()

            val tags = buildSet {
                for (i in 1..tagCount) {
                    if ((value and (1 shl i - 1)) != 0) {
                        add(i)
                    }
                }
            }

            return AprilTagList(tags)
        }
    }

    companion object {
        @JvmField val struct = AprilTagListStruct()
    }
}
