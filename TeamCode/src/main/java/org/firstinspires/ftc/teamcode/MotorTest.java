package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Basic: Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor activeMotor = null;
    // To test more motors, add them to the below array
    private String[] motorNames = {"ladder_motor", "intake_motor", "left_front", "left_back", "right_front", "right_back"};
    private int activeMotorIndex = 0;

    private void setMotor() {
        setMotor(0);
    }

    private void setMotor(int delta) {
        activeMotorIndex += delta;
        activeMotorIndex = (activeMotorIndex + motorNames.length) % motorNames.length;
        activeMotor = hardwareMap.get(DcMotor.class, motorNames[activeMotorIndex]);
        activeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void runOpMode() {
        setMotor();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        final int LOW_SPEED = 33;
        final int MEDIUM_SPEED = 66;
        final int HIGH_SPEED = 100;

        int motorPower = MEDIUM_SPEED;

        boolean lastFramedDpadLeft = false;
        boolean lastFramedDpadRight = false;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motorPower = LOW_SPEED;
            } else if (gamepad1.b) {
                motorPower = MEDIUM_SPEED;
            } else if (gamepad1.y) {
                motorPower = HIGH_SPEED;
            } else if (gamepad1.x) {
                motorPower = 50; // I know it's a magic number, but it's worse to call it HALF_SPEED or similar, isn't it?
            }

            if (gamepad1.right_bumper) {
                motorPower += 10;
            } else if (gamepad1.left_bumper) {
                motorPower -= 10;
            }

            if (motorPower < 0) {
                motorPower = 0;
            } else if (motorPower > 100) {
                motorPower = 100;
            }

            lastFramedDpadLeft = gamepad1.dpad_left;
            lastFramedDpadRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !lastFramedDpadLeft) {
                setMotor(-1);
            } else if (gamepad1.dpad_right && !lastFramedDpadRight) {
                setMotor(1);
            }

            if (gamepad1.dpad_up) {
                activeMotor.setPower(-motorPower);
            } else if (gamepad1.dpad_down) {
                activeMotor.setPower(motorPower);
            } else {
                activeMotor.setPower(0);
            }

            telemetry.addData(motorNames[activeMotorIndex]+" power: ", motorPower);
            telemetry.update();
        }
    }
}
