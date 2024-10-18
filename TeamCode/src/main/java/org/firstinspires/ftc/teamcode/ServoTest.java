package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Basic: Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Servo activeServo = null;
    // To test more servos, add them to the below array
    private String[] servoNames = {"extend_servo", "bucket_servo", "intake_servo", "claw_servo", "claw_wrist_servo"};
    private int activeServoIndex = 0;

    private void setServo() {
        setServo(0);
    }

    private void setServo(int delta) {
        activeServoIndex += delta;
        activeServoIndex = (activeServoIndex + servoNames.length) % servoNames.length;
        activeServo = hardwareMap.get(Servo.class, servoNames[activeServoIndex]);
    }

    @Override
    public void runOpMode() {
        setServo();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int activeServoPosition = 0;

        boolean lastFrameDpadLeft = false;
        boolean lastFrameDpadRight = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                activeServoPosition = 0;
            } else if (gamepad1.b) {
                activeServoPosition = 1;
            }

            if (gamepad1.dpad_left && !lastFrameDpadLeft) {
                setServo(-1);
            } else if (gamepad1.dpad_right && !lastFrameDpadRight) {
                setServo(1);
            }
            lastFrameDpadLeft = gamepad1.dpad_left;
            lastFrameDpadRight = gamepad1.dpad_right;

            if (gamepad1.dpad_up) {
                activeServoPosition = 0;
            } else if (gamepad1.dpad_down) {
                activeServoPosition = 1;
            }

            activeServo.setPosition(activeServoPosition);

            telemetry.addData(servoNames[activeServoIndex]+ " position: ", activeServoPosition);
            telemetry.update();
        }
    }
}
