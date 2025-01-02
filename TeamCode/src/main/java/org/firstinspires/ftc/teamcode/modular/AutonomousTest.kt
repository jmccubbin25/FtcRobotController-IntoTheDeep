package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.pow
import kotlin.math.sign

@Suppress("unused")
@Autonomous(name = "Autonomous Test", group = "Auto", preselectTeleOp = "DriveTrain")
class AutonomousTest : OpMode() {
    private lateinit var leftBackMotor: DcMotorEx
    private lateinit var rightBackMotor: DcMotorEx
    private lateinit var rightFrontMotor: DcMotorEx
    private lateinit var leftFrontMotor: DcMotorEx
    private lateinit var odometry: GoBildaPinpointDriver
    private lateinit var motors: Array<DcMotorEx>

    override fun init() {
        this.telemetry.msTransmissionInterval = 10
        try {
            this.leftBackMotor = this.hardwareMap.dcMotor["leftRear"] as DcMotorEx
            this.rightBackMotor = this.hardwareMap.dcMotor["rightRear"] as DcMotorEx
            this.rightFrontMotor = this.hardwareMap.dcMotor["rightFront"] as DcMotorEx
            this.leftFrontMotor = this.hardwareMap.dcMotor["leftFront"] as DcMotorEx

            this.leftFrontMotor.direction = DcMotorSimple.Direction.REVERSE
            this.rightFrontMotor.direction = DcMotorSimple.Direction.FORWARD
            this.leftBackMotor.direction = DcMotorSimple.Direction.REVERSE
            this.rightBackMotor.direction = DcMotorSimple.Direction.FORWARD

            this.motors = arrayOf(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor)
            this.motors.forEach {
                it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            }

            this.odometry = this.hardwareMap.get(GoBildaPinpointDriver::class.java, "odometry")
            this.odometry.setEncoderResolution(37.25135125)
            this.odometry.setOffsets(95.0, 0.0)
            this.odometry.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
            )
            odometry.resetPosAndIMU()
        } catch (e: Exception) {
        }
    }

    private enum class Mode(vararg val mappings: Int) {
        RUN(1, 1, 1, 1),
        TURN(1, -1, 1, -1),
        STRAFE(-1, -1, 1, 1);

        fun mappings(): IntArray = mappings
    }

    private enum class Stage(val targetGetter: (AutonomousTest, GoBildaPinpointDriver) -> Target) {
        ONE({ robot, odometry -> robot.Target(Mode.RUN, 500, odometry.position) }),
        TWO({ robot, odometry -> robot.Target(Mode.TURN, 90, odometry.position) }),
        THREE({ robot, odometry -> robot.Target(Mode.STRAFE, 500, odometry.position) });

        private lateinit var target: Target
        private var started = false
        private var finished = false

        fun start(inst: AutonomousTest) {
            target = targetGetter(inst, inst.odometry)
            started = true
        }

        fun finish() {
            finished = true
        }

        fun check(pos: Pose2D) {
            if (!finished() && target.check(pos)) {
                finish()
            }
        }

        fun finished() = finished
        fun started() = started
        fun target() = target
    }

    private inner class Target(
        private val mode: Mode,
        private val target: Int,
        private val start: Pose2D
    ) {
        init {
            assert(mode != Mode.TURN || abs(target) < 360)
        }

        var last_h = start.getHeading(AngleUnit.DEGREES)
        val start_pos = start.getX(DistanceUnit.MM).pow(2) + start.getY(DistanceUnit.MM).pow(2)

        fun mode() = mode
        fun target_num() = target

        fun check(pos: Pose2D): Boolean {
            return when (mode) {
                Mode.RUN, Mode.STRAFE -> {
                    val curr = pos.getX(DistanceUnit.MM).pow(2) + pos.getY(DistanceUnit.MM).pow(2)
                    telemetry.addData("Start", start_pos)
                    telemetry.addData("Current", curr)
                    curr >= target * target + start_pos
                }

                Mode.TURN -> {
                    fun fix(heading: Double): Double {
                        return if (heading < 0) abs(heading)
                        else 180 + abs(heading - 180)
                    }

                    val curr_h = fix(pos.getHeading(AngleUnit.DEGREES))
                    val start_h = fix(start.getHeading(AngleUnit.DEGREES))
                    telemetry.addData("Last Heading", floor(last_h))
                    telemetry.addData("Current Heading", floor(curr_h))
                    telemetry.addData("Target", target)
                    assert(last_h != curr_h)
                    telemetry.addData("data", target > 0 && ((last_h < target && curr_h >= target)))
                    if ((target < 0 && ((last_h > target && curr_h <= target) || (curr_h > start_h + 10 && last_h > target && curr_h - 360 <= target))) ||
                        (target > 0 && ((last_h < target && curr_h >= target) || (curr_h < start_h - 10 && last_h < target && curr_h + 360 >= target)))
                    ) {
                        return true
                    }
                    last_h = curr_h
                    false
                }
            }
        }
    }

    private var stage = Stage.ONE

    private fun reset() {
        this.motors.forEach { it.power = 0.0 }
    }

    override fun loop() {
        this.odometry.update()
        this.telemetry.addData("Stage", this.stage)

        if (!this.stage.started()) {
            this.stage.start(this)
            val sign = this.stage.target().target_num().sign
            this.stage.target().mode().mappings().zip(this.motors)
                .forEach { (dir: Int, motor: DcMotorEx) -> motor.power = 0.3 * dir * sign }
        }

        this.stage.check(this.odometry.position)

        if (this.stage.finished()) {
            if (this.stage.ordinal < Stage.entries.size - 1) {
                this.stage = Stage.entries[this.stage.ordinal + 1]
                reset()
            }
        }

        this.telemetry.update()
    }
}
