package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Basic: Custom Holonomic Drive", group="Linear OpMode")
public class CustomHolonomicDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor viperSlideMotor = null;

//    private DcMotor xEncoder = null; // If uncommented and not attached to the robot, things will break
//    private DcMotor yEncoder = null;

    private Servo extendServo = null;
    private Servo bucketServo = null;
    private Servo intakeServo = null;
    private Servo clawServo = null;
    private Servo clawWristServo = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

//        xEncoder = hardwareMap.get(DcMotor.class, "x_encoder");
//        yEncoder = hardwareMap.get(DcMotor.class, "y_encoder");

        extendServo = hardwareMap.get(Servo.class, "extend_servo");
        bucketServo = hardwareMap.get(Servo.class, "bucket_servo");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawWristServo = hardwareMap.get(Servo.class, "claw_wrist_servo");
        clawWristServo.scaleRange(0.0, 0.5); // Normal 0 is too far down


        viperSlideMotor = hardwareMap.get(DcMotor.class, "viper_slide_motor");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        viperSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            double extendServoPosition = 0.0;
            double bucketServoPosition = 0.0;
            double intakeServoPosition = 0.0;
            double clawServoPosition = 0.0;
            double clawWristServoPosition = 0.0; // Servo left value is too far down, center is perfect

            int viperSlideMotorPower = 0;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double axialThreshold = 0.05 * lateral;
            double lateralThreshold = 0.05 * axial;

            // If not steering much, assume it's because of human inaccuracy and fix it (untested)
            if (Math.abs(axial) <= axialThreshold) {
                axial = 0;
            } else if (Math.abs(lateral) <= lateralThreshold) {
                lateral = 0;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Viper slide motor logic
            if (gamepad1.right_trigger > 0) { // Put deadzone later
                viperSlideMotorPower = Math.round(gamepad1.right_trigger * 100);
            } else if (gamepad1.left_trigger > 0) {
                viperSlideMotorPower = Math.round(-(gamepad1.left_trigger * 100));
            } else {
                viperSlideMotorPower = 0;
            }

            // Servo position logic
            if (gamepad1.a && extendServoPosition == 0.0) {
                extendServoPosition = 1.0;
            } else if (gamepad1.a && extendServoPosition == 1.0) {
                extendServoPosition = 0.0;
            }

            if (gamepad1.b && bucketServoPosition == 0.0) {
                bucketServoPosition = 1.0;
            } else if (gamepad1.b && bucketServoPosition == 1.0) {
                bucketServoPosition = 0.0;
            }

            if (gamepad1.y && intakeServoPosition == 0.0) {
                intakeServoPosition = 1.0;
            } else if (gamepad1.y && intakeServoPosition == 1.0) {
                intakeServoPosition = 0.0;
            }

            if (gamepad1.x && clawServoPosition == 0.0) {
                clawServoPosition = 1.0;
            } else if (gamepad1.x && clawServoPosition == 1.0) {
                clawServoPosition = 0.0;
            }

            if (gamepad1.right_bumper && clawWristServoPosition == 0.0) {
                clawWristServoPosition = 1.0;
            } else if (gamepad1.right_bumper && clawWristServoPosition == 1.0) {
                clawWristServoPosition = 0.0;
            }

            // Set servo positions
            extendServo.setPosition(extendServoPosition);
            bucketServo.setPosition(bucketServoPosition);
            intakeServo.setPosition(intakeServoPosition);
            clawServo.setPosition(clawServoPosition);
            clawWristServo.setPosition(clawWristServoPosition);

            // Set (non-drive) motor power
            viperSlideMotor.setPower(viperSlideMotorPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("xEncoder", "%4.2f, %4.2f", xEncoder);
//            telemetry.addData("yEncoder", "%4.2f, %4.2f", yEncoder);
            telemetry.addData("extendServo position: ", extendServoPosition);
            telemetry.addData("bucketServo position: ", bucketServoPosition);
            telemetry.addData("intakeServo position: ", intakeServoPosition);
            telemetry.addData("clawServo position: ", clawServoPosition);
            telemetry.addData("clawWristServo position: ", clawWristServoPosition);
            telemetry.addData("viperSlideMotorPower", viperSlideMotorPower);
            telemetry.update();
        }
    }
}