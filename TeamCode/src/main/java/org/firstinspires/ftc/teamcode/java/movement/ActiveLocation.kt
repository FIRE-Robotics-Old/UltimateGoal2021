package org.firstinspires.ftc.teamcode.java.movement

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.java.util.Angle

class ActiveLocation: Runnable {

    /**
     * IMU to Detect Angle Movement
     */
    private val imu: BNO055IMU

    /**
     * Y Direction Encoder to Detect Y-Axis Movement
     */
    private val yDirectionEncoder: DcMotorEx

    /**
     * X Direction Encoder to Detect X-Axis Movement
     */
    private val xDirectionEncoder: DcMotorEx

    /**
     * Internal Y Encoder Value
     *
     * This value factors in angle rotations and movements
     */
    @Volatile
    private var internalCurrentY: Double = 0.0

    /**
     * Internal X Encoder Value
     *
     * This value factors in angle and rotation movements
     */
    @Volatile
    private var internalCurrentX: Double = 0.0

    /**
     * The value bound to the Y Axis Encoder
     */
    private var yEncoder: Double = 0.0

    /**
     * The value bound to the X Axis Encoder
     */
    private var xEncoder: Double = 0.0

    /**
     * The Current Angular Position of the Robot (As an {@link Angle})
     */
    var angle: Angle = Angle.fromRadians(0.0)

    // TODO: Combine Start Angle and Reset Angle
    /**
     * The Angle to Reset the Robot to When Changing the Field References
     */
    var resetAngle = Angle.fromRadians(0.0)

    /**
     * The Initial Angle of rht Robot for Initialization
     */
    var startAngle: Angle = Angle.fromRadians(0.0)

    /**
     * Robot's Absolute X-Axis Position on the Field
     */
    @Volatile
    var fieldPositionX: Double = 0.0

    /**
     * Robot's Absolute Y-Axis Position on the Field
     */
    @Volatile
    var fieldPositionY: Double = 0.0

    /**
     * Whether or not the thread is still running
     */
    @Volatile
    private var isRunning: Boolean = true

    override fun run() {
        TODO("Not yet implemented")
    }
}