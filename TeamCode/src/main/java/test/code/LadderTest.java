package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Basic: Ladder Test", group = "Test")
public class LadderTest extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor = null;

    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotor.class, "ladder_motor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            final int LOW_SPEED = 33;
            final int MEDIUM_SPEED = 66;
            final int HIGH_SPEED = 100;

            int motorPower = MEDIUM_SPEED;

            if (gamepad1.a) {
                motorPower = LOW_SPEED;
            } else if (gamepad1.b) {
                motorPower = MEDIUM_SPEED;
            } else if (gamepad1.y) {
                motorPower = HIGH_SPEED;
            } else if (gamepad1.x) {
                motorPower = 50; // I know it's a magic number, but it's worse to call it MEDIUM_SPEED or similar, isn't it?
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

            if (gamepad1.dpad_up) {
                intakeMotor.setPower(motorPower);
            } else if (gamepad1.dpad_down) {
                intakeMotor.setPower(-motorPower);
            }

            telemetry.addData("Wheel power", "%4.2f, %4.2f", motorPower);
            telemetry.update();
        }
    }
}
