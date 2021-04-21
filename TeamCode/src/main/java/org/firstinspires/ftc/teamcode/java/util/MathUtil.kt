package org.firstinspires.ftc.teamcode.java.util

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

object MathUtil {
    fun squared(num: Double): Double {
        return num * num
    }

    fun squared(num: Float): Float {
        return num * num
    }

    fun squared(num: Byte): Int {
        return num * num
    }

    fun squared(num: Short): Int {
        return num * num
    }

    fun squared(num: Int): Long {
        return (num * num).toLong()
    }

    @JvmStatic
    fun squared(num: Long): Long {
        return num * num
    }

    fun cube(num: Double): Double {
        return num * num * num
    }

    fun cube(num: Float): Float {
        return num * num * num
    }

    fun cube(num: Byte): Int {
        return num * num * num
    }

    fun cube(num: Short): Int {
        return num * num * num
    }

    fun cube(num: Int): Long {
        return (num * num * num).toLong()
    }

    fun cube(num: Long): Long {
        return num * num * num
    }

    fun pow(_num: Double, _times: Int): Double {
        var num = _num
        var times = _times
        var sol = 1.0
        while (times > 0) {
            if (times and 1 == 1) sol *= num
            num *= num
            times = times shr 1
        }
        return sol
    }

    fun pow(_num: Float, _times: Int): Float {
        var num = _num
        var times = _times
        var sol = 1f
        while (times > 0) {
            if (times and 1 == 1) sol *= num
            num *= num
            times = times shr 1
        }
        return sol
    }

    fun pow(_num: Byte, _times: Int): Int {
        var num: Int = _num.toInt()
        var times: Int = _times
        var sol = 1
        while (times > 0) {
            if (times and 1 == 1) sol *= num
            num *= num
            times = times shr 1
        }
        return sol
    }

    fun pow(_num: Short, _times: Int): Int {
        var num: Int = _num.toInt()
        var times: Int = _times
        var sol = 1
        while (times > 0) {
            if (times and 1 == 1) sol *= num
            num *= num
            times = times shr 1
        }
        return sol
    }

    fun pow(_num: Int, _times: Int): Long {
        var num = _num
        var times = _times
        var sol = 1L
        while (times > 0) {
            if (times and 1 == 1) sol *= num.toLong()
            num *= num
            times = times shr 1
        }
        return sol
    }

    fun powInt(_num: Int, _times: Int): Int {
        var num: Int = _num
        var times: Int = _times
        var sol = 1
        while (times > 0) {
            if (times and 1 == 1) sol *= num
            num *= num
            times = times shr 1
        }
        return sol
    }

    fun pow(_num: Long, _times: Long): Long {
        var num: Long = _num
        var times: Long = _times
        var sol: Long = 1
        while (times > 0) {
            if (times and 1 == 1L) sol *= num
            num *= num
            times = times shr 1
        }
        return sol
    }

    fun sin(angle: Angle): Double {
        return sin(angle.angleInRadians)
    }

    fun cos(angle: Angle): Double {
        return cos(angle.angleInRadians)
    }

    fun tan(angle: Angle): Double {
        return tan(angle.angleInRadians)
    }
}