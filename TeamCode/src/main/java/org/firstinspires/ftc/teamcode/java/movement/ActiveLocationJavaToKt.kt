package org.firstinspires.ftc.teamcode.java.movement

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.java.util.Angle
import org.firstinspires.ftc.teamcode.java.util.Constants
import org.firstinspires.ftc.teamcode.java.util.MovementData
import org.firstinspires.ftc.teamcode.java.util.RobotHardware

/**
 * The ActiveLocation uses odometry to find the real-time location of the robot. This, along with a
 * PathFinder, helps create a Field Mapping to allow us to accurately move to specific positions in
 * autonomous.
 */
class ActiveLocationJavaToKt : Runnable {
    // Hardware setup
    private val imu: BNO055IMU
    private val yDirectionEncoder: DcMotor
    private val xDirectionEncoder: DcMotor

    // ########## VARIABLE SET UP ##########\\
    // X and Y based on encoder
    @Volatile
    private var internalCurrentY = 0.0

    @Volatile
    private var internalCurrentX = 0.0

    // Used for sensor values
    private var yEncoder = 0.0
    private var xEncoder = 0.0
    var angle = Angle.fromRadians(0.0)
    var resetAngle = Angle.fromRadians(0.0)
    var startAngle = Angle.fromRadians(0.0)

    // Field location
    var fieldXPosition = 0.0
    var fieldYPosition = 0.0

    // For stopping the thread
    @Volatile
    private var isRunning = true

    /**
     * Creates an Active Location Tracker for the Robot given by a [RobotHardware]
     *
     * @param robot the HardwareMap set of the Robot
     */
    constructor(robot: RobotHardware) {
        yDirectionEncoder = robot.frontLeftMotor
        xDirectionEncoder = robot.backRightMotor
        imu = robot.imu
    }

    /**
     * Create an Active Location Tracker for the Robot
     *
     * @param xDirectionEncoder the encoder set up in the X Direction
     * @param yDirectionEncoder the encoder set up in the Y Direction
     * @param gyroscope the imu (aka gyroscope) to determine the angle of the robot
     */
    constructor(xDirectionEncoder: DcMotor, yDirectionEncoder: DcMotor,
                gyroscope: BNO055IMU) {
        this.yDirectionEncoder = yDirectionEncoder
        this.xDirectionEncoder = xDirectionEncoder
        imu = gyroscope
    }

    /**
     * Sets the start position
     *
     * @param startX The current X position mm
     * @param startY The current Y position mm
     * @param startAngle The starting angle
     */
    fun setStartPosition(startX: Double, startY: Double, startAngle: Angle) {
        this.startAngle = startAngle
        internalCurrentX = (startX * Math.cos(startAngle.angleInRadians)
                - startY * Math.sin(startAngle.angleInRadians))
        internalCurrentY = (startX * Math.sin(startAngle.angleInRadians)
                + startY * Math.cos(startAngle.angleInRadians))
    }

    /**
     * Sets the start position
     *
     * @param location A movement data containing x and y position in mm and the angle in degrees
     */
    fun setStartPosition(location: MovementData) {
        this.setStartPosition(location.x, location.y, location.angle)
    }

    /**
     * Updates Relevant Values for ActiveLocation calculations
     *
     * This function updates both the X and Y values of the Encoder's position, as well as the angle
     * of the robot using the built in IMU on the Rev Hub.
     */
    private fun updateSensors() {
        yEncoder = yDirectionEncoder.currentPosition.toDouble()
        xEncoder = xDirectionEncoder.currentPosition.toDouble()
        angle = Angle.fromRadians(-(imu.angularOrientation.firstAngle
                + startAngle.angleInRadians - resetAngle.angleInRadians))
        // angle = ((angle + (2 * Math.PI)) % (2 * Math.PI));
    }
    // "-" is there because directions are backwards /
    /**
     * Updates the current position of the Robot on the Field
     *
     * This function calculates the change in the robot's positional values, recalculating the
     * position of the robot.
     */
    fun findFieldPosition() {
        val internalPreviousY = internalCurrentY
        val internalPreviousX = internalCurrentX
        synchronized(this) {
            internalCurrentY = tickToDistance(yEncoder)
            internalCurrentX = tickToDistance(xEncoder)
        }
        // Change in internal x and y values
        val deltaY = internalCurrentY - internalPreviousY
        val deltaX = internalCurrentX - internalPreviousX
        fieldXPosition += (deltaX * Math.cos(angle.angleInRadians)
                - deltaY * Math.sin(angle.angleInRadians))
        fieldYPosition += (deltaX * Math.sin(angle.angleInRadians)
                + deltaY * Math.cos(angle.angleInRadians))
    }

    /**
     * Gets the Robot's Position on the field on the Y-Axis
     *
     * @return Returns the robot's Y position on the field (mm)
     */
    val fieldY: Double
        get() {
            updateSensors()
            findFieldPosition()
            return fieldYPosition
        }// return startX;

    /**
     * Gets the Robot's Position on the field on the X-Axis
     *
     * @return returns X position on the field (mm)
     */
    val fieldX: Double
        get() {
            updateSensors()
            findFieldPosition()
            // return startX;
            return fieldXPosition
        }

    /**
     * Get's the Robot's Angle on the field, in degrees
     *
     * @return returns angle in degrees
     */
    val angleInRadians: Double
        get() {
            updateSensors()
            return angle.angleInRadians
        }

    /**
     * Get's the Robot's Angle on the field, in degrees
     *
     * @return returns angle in degrees
     */
    val angleInDegrees: Double
        get() {
            updateSensors()
            return angle.angleInDegrees
        }

    /**
     * Takes the current angle and sets that value to 0 in the robots point of view
     */
    fun resetAngle() {
        updateSensors()
        resetAngle = angle
    }

    /**
     * Sets up the thread to stop
     */
    fun stop() {
        isRunning = false
    }

    /**
     * Sets up and starts the thread running in the background
     *
     * This function determines what the class will do when set to run on an individual thread,
     * maintaining the current position of the robot so that autonomous can update and properly
     * calculate where the robot needs to go and when the robot reached said position.
     */
    override fun run() {
        while (isRunning) {
            updateSensors()
            findFieldPosition()
        }
    }

    companion object {
        // Static values for calculations
        const val tickPerRotation = 8192.0
        const val wheelCircumference = 90 * Constants.PI

        /**
         * Converts Ticks to Distance
         *
         * To calculate the distance, this function calculates the fraction of a total rotation (which
         * can be greater than 1 rotation) and multiplies it by the circumference, getting the linear
         * distance from the rotational fraction.
         *
         * @param ticks tick value
         * @return mm distance value
         */
        private fun tickToDistance(ticks: Double): Double {
            return ticks / tickPerRotation * wheelCircumference
        }

        /**
         * Converts Distance to Ticks
         *
         * To calculate the ticks traveled, the function determines the fraction of a rotation (using
         * distance and the circumference) and multiplies by the constant ticksPerRotation, returning
         * the total ticks.
         *
         * @param distance mm value
         * @return returns tick value
         */
        @Deprecated("")
        private fun distanceToTicks(distance: Double): Double {
            return distance / wheelCircumference * tickPerRotation
        }
    }
}