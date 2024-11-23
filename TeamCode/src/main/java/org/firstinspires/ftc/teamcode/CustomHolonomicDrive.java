package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import common.SimpleDrive;

@TeleOp(name = "Basic: Custom Holonomic Drive", group = "Linear OpMode")
public class CustomHolonomicDrive extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu = null;

    private DcMotor viperSlideMotor = null;

    private DcMotor leftHangingMotor = null;
    private DcMotor rightHangingMotor = null;

    private Servo extendServo = null;
    private Servo bucketServo = null;
    private Servo intakeServo = null;
    private Servo sampleClawServo = null;
    private Servo wristServo = null;
    private Servo specimenClawServo = null;
    private Servo leftHangingServo = null;
    private Servo rightHangingServo = null;

    enum State {
        IDLE,
        EXTENDED,
        PLACE_SPECIMEN_HIGH_BAR,
        PLACE_SPECIMEN_LOW_BAR,
        GRABBED,
        LOADED,
        LIFTED_HIGH_BUCKET,
        LIFTED_LOW_BUCKET,
        DROP_HIGH_BUCKET,
        DROP_LOW_BUCKET,
        HANGING,
    }

    private State state = State.IDLE;

    @Override
    public void runOpMode() {
        SimpleDrive drive = new SimpleDrive(this);
        drive.start();

        imu = hardwareMap.get(IMU.class, "imu");

        extendServo = hardwareMap.get(Servo.class, "extend_servo");
        bucketServo = hardwareMap.get(Servo.class, "bucket_servo");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        sampleClawServo = hardwareMap.get(Servo.class, "sample_claw_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        specimenClawServo = hardwareMap.get(Servo.class, "specimen_claw_servo");
        leftHangingServo = hardwareMap.get(Servo.class, "left_hanging_servo");
        rightHangingServo = hardwareMap.get(Servo.class, "right_hanging_servo");

        viperSlideMotor = hardwareMap.get(DcMotor.class, "viper_slide_motor");
        leftHangingMotor = hardwareMap.get(DcMotor.class, "left_hanging_motor");
        rightHangingMotor = hardwareMap.get(DcMotor.class, "right_hanging_motor");

        viperSlideMotor.setTargetPosition(0);
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideMotor.setPower(1);
        viperSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        leftHangingMotor.setTargetPosition(0);
        leftHangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftHangingMotor.setPower(1);
        leftHangingMotor.setDirection(DcMotor.Direction.FORWARD);
        rightHangingMotor.setTargetPosition(0);
        rightHangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightHangingMotor.setPower(1);
        rightHangingMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double liftedTime = 0;
        double wristTime = 0;
        double closedTime = 0;

        double extendServoPosition = 0.0;
        double bucketServoPosition = 0.0;
        double intakeServoPosition = 0.0;
        double sampleClawServoPosition = 0.0;
        double wristServoPosition = 0.0;
        double specimenClawServoPosition = 0.0;

        double hangingServoPosition = 0.0;
        double

        int viperSlideMotorPosition = 0;

        int liftDown = 0;
        int liftTopBucket = 6180;
        int liftBottomBucket = 3480;
        int liftTopBar = 2890;
        int liftBottomBar = 960;

        double bucketDrop = 0.37;
        double bucketLoad = 0.5;

        int leftHangingMotorPosition = 0;
        int rightHangingMotorPosition = 0;

        int hangingMotorDown = 0;
        int hangingMotorOut = ;

        double extendClosed = 0.0;
        double extendExtended = 1.0;

        double intakeDown = 0.26;
        double intakeUp = 1;

        double wristLoad = 0.5;
        double wristDrop = 1;
        double wristLift = 0.2;

        double sampleClawClosed = 0;
        double sampleClawOpen = 0.4;

        double specimenClawClosed = 0;
        double specimenClawOpen = 0.5;

        boolean aPrev = false;
        boolean bPrev = false;
        boolean xPrev = false;
        boolean yPrev = false;
        boolean rbPrev = false;

        boolean dpadUpPrev = false;
        boolean dpadRightPrev = false;
        boolean dpadDownPrev = false;
        boolean dpadLeftPrev = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean a = gamepad1.a && !aPrev;
            aPrev = gamepad1.a;
            boolean b = gamepad1.b && !bPrev;
            bPrev = gamepad1.b;
            boolean x = gamepad1.x && !xPrev;
            xPrev = gamepad1.x;
            boolean y = gamepad1.y && !yPrev;
            yPrev = gamepad1.y;
            boolean rb = gamepad1.right_bumper && !rbPrev;
            rbPrev = gamepad1.right_bumper;

            boolean dpadUp = gamepad1.dpad_up && !dpadUpPrev;
            dpadUpPrev = gamepad1.dpad_up;
            boolean dpadRight = gamepad1.dpad_right && !dpadRightPrev;
            dpadRightPrev = gamepad1.dpad_right;
            boolean dpadDown = gamepad1.dpad_down && !dpadDownPrev;
            dpadDownPrev = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left && !dpadLeftPrev;
            dpadLeftPrev = gamepad1.dpad_left;

            switch (state) {
                case IDLE:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    wristServoPosition = wristLift;
                    sampleClawServoPosition = sampleClawClosed;

                    if (runtime.seconds() > closedTime + 0.5) {
                        specimenClawServoPosition = specimenClawOpen;
                    }

                    if (a) {
                        state = State.EXTENDED;
                    } else if (y) {
                        state = State.PLACE_SPECIMEN_HIGH_BAR;
                    } else if (x) {
                        state = State.PLACE_SPECIMEN_LOW_BAR;
                    }
                    break;
                case EXTENDED:
                    if (runtime.seconds() > wristTime + 0.5) {
                        sampleClawServoPosition = sampleClawOpen;
                    }
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeDown;
                    wristServoPosition = wristLoad;
                    specimenClawServoPosition = specimenClawClosed;

                    if (a) {
                        state = State.GRABBED;
                    } else if (b) {
                        state = State.IDLE;
                    }
                    break;
                case PLACE_SPECIMEN_HIGH_BAR:
                    viperSlideMotorPosition = liftTopBar;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    wristServoPosition = wristLoad;
                    sampleClawServoPosition = sampleClawClosed;
                    specimenClawServoPosition = specimenClawClosed;

                    if (a || b) {
                        state = State.IDLE;
                        closedTime = runtime.seconds();
                    }
                    break;
                case PLACE_SPECIMEN_LOW_BAR:
                    viperSlideMotorPosition = liftBottomBar;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    wristServoPosition = wristLoad;
                    sampleClawServoPosition = sampleClawClosed;
                    specimenClawServoPosition = specimenClawClosed;

                    if (a || b) {
                        state = State.IDLE;
                        closedTime = runtime.seconds();
                    }
                    break;
                case GRABBED:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeDown;
                    wristServoPosition = wristLoad;
                    sampleClawServoPosition = sampleClawClosed;
                    specimenClawServoPosition = specimenClawClosed;

                    if (a) {
                        state = State.LOADED;
                    } else if (b) {
                        state = State.EXTENDED;
                    } else if (rb) {
                        state = State.IDLE;
                    }
                    break;
                case LOADED:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    wristServoPosition = wristDrop;
                    sampleClawServoPosition = sampleClawClosed;
                    specimenClawServoPosition = specimenClawClosed;

                    if (y) {
                        state = State.LIFTED_HIGH_BUCKET;
                        liftedTime = runtime.seconds();
                    } else if (x) {
                        state = State.LIFTED_LOW_BUCKET;
                        liftedTime = runtime.seconds();
                    } else if (b) {
                        state = State.GRABBED;
                        wristTime = runtime.seconds();
                    } else if (rb) {
                        state = State.IDLE;
                    }
                    break;
                case LIFTED_HIGH_BUCKET:
                    if (runtime.seconds() > liftedTime + 1) {
                        viperSlideMotorPosition = liftTopBucket;
                    }

                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    specimenClawServoPosition = specimenClawClosed;

                    if (runtime.seconds() > liftedTime + 0.5) {
                        wristServoPosition = wristLift;
                    }
                    sampleClawServoPosition = sampleClawOpen;

                    if (a) {
                        state = State.DROP_HIGH_BUCKET;
                    } else if (b) {
                        state = State.IDLE;
                    }
                    break;
                case LIFTED_LOW_BUCKET:
                    if (runtime.seconds() > liftedTime + 1) {
                        viperSlideMotorPosition = liftBottomBucket;
                    }
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    specimenClawServoPosition = specimenClawClosed;

                    if (runtime.seconds() > liftedTime + 0.5) {
                        wristServoPosition = wristLift;
                    }
                    sampleClawServoPosition = sampleClawOpen;

                    if (a) {
                        state = State.DROP_LOW_BUCKET;
                    } else if (b) {
                        state = State.IDLE;
                    }
                    break;
                case DROP_HIGH_BUCKET:
                    viperSlideMotorPosition = liftTopBucket;
                    bucketServoPosition = bucketDrop;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    wristServoPosition = wristLift;
                    sampleClawServoPosition = sampleClawOpen;
                    specimenClawServoPosition = specimenClawClosed;

                    if (a) {
                        state = State.IDLE;
                    } else if (b) {
                        state = State.LIFTED_HIGH_BUCKET;
                    }
                    break;
                case DROP_LOW_BUCKET:
                    viperSlideMotorPosition = liftBottomBucket;
                    bucketServoPosition = bucketDrop;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    wristServoPosition = wristLift;
                    sampleClawServoPosition = sampleClawOpen;
                    specimenClawServoPosition = specimenClawClosed;

                    if (a) {
                        state = State.IDLE;
                    }
                    if (b) {
                        state = State.LIFTED_LOW_BUCKET;
                    }
                    break;
                case HANGING:

                    break;
            }

            // Set servo positions
            extendServo.setPosition(extendServoPosition);
            bucketServo.setPosition(bucketServoPosition);
            intakeServo.setPosition(intakeServoPosition);
            sampleClawServo.setPosition(sampleClawServoPosition);
            wristServo.setPosition(wristServoPosition);
            specimenClawServo.setPosition(specimenClawServoPosition);

            // Set (non-drive) motor power
            viperSlideMotor.setTargetPosition(viperSlideMotorPosition);
            if (viperSlideMotor.isBusy()) {
                viperSlideMotor.setPower(1);
            } else if (!viperSlideMotor.isBusy() && viperSlideMotorPosition == 0) {
                viperSlideMotor.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("State", state);
            telemetry.addData("extendServo position: ", extendServoPosition);
            telemetry.addData("bucketServo position: ", bucketServoPosition);
            telemetry.addData("intakeServo position: ", intakeServoPosition);
            telemetry.addData("clawServo position: ", sampleClawServoPosition);
            telemetry.addData("clawWristServo position: ", wristServoPosition);
            telemetry.addData("SpecimenClawServoPosition", specimenClawServoPosition);
            telemetry.addData("Viper encoder: ", viperSlideMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
