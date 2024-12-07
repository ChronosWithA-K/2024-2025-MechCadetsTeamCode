package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Test Path", group = "Linear OpMode")
public class TestAutoPath extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor viperSlideMotor = null;

    private Servo specimenClawServo = null;
    private Servo bucketServo = null;
    private Servo intakeServo = null;

    static final double COUNTS_PER_MOTOR_REV = 435;
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.09449; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.5;

    int liftTopBar = 2890;
    int engageClaw = 2290;

    @Override
    public void runOpMode() {
        double specimenClawClosed = 0;
        double specimenClawOpen = 0.5;
        double bucketLoad = 0.5;
        double intakeIdle = 0.65;

        // Initialize the hardware variables
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viperSlideMotor = hardwareMap.get(DcMotor.class, "viper_slide_motor");
        viperSlideMotor.setTargetPosition(0);
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideMotor.setPower(1);
        viperSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        specimenClawServo = hardwareMap.get(Servo.class, "specimen_claw_servo");
        bucketServo = hardwareMap.get(Servo.class, "bucket_servo");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");

        bucketServo.setPosition(bucketLoad);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            bucketServo.setPosition(bucketLoad);

            if (runtime.seconds() < 3) {
                telemetry.addLine("Stage 1");
                specimenClawServo.setPosition(specimenClawClosed);
                viperSlideMotor.setTargetPosition(liftTopBar);
                intakeServo.setPosition(intakeIdle);
                encoderDrive(DRIVE_SPEED, -4, -4, -4, -4);
            } else if (runtime.seconds() < 5 && runtime.seconds() >= 3) {
                telemetry.addLine("Stage 2");
                viperSlideMotor.setTargetPosition(engageClaw);
                bucketServo.setPosition(bucketLoad);
                resetEncoders();
            } else if (runtime.seconds() < 8 && runtime.seconds() >= 5) {
                telemetry.addLine("Stage 3");
                specimenClawServo.setPosition(specimenClawOpen);
                viperSlideMotor.setTargetPosition(0);
            } else if (runtime.seconds() < 14 && runtime.seconds() >= 12) {
                telemetry.addLine("Stage 4");
                encoderDrive(DRIVE_SPEED, -24, 24, 24, -24); // Strafe right 2 squares
            } else {
                telemetry.addLine("Finished");
            }
            telemetry.addData("runtime.Seconds(): ", runtime.seconds());
            telemetry.update();
        }
    }

    public void resetEncoders(){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void encoderDrive(double speed, double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Determine new target position and pass to motor controller
        newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
        newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
        newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
        newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

        // Set target positions
        leftFrontDrive.setTargetPosition(newLeftFrontTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightFrontDrive.setTargetPosition(newRightFrontTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        // Turn on RUN_TO_POSITION mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the timeout time and start motion
        leftFrontDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));
    }
}
