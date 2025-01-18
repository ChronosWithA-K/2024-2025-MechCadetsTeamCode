package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import common.SimpleDrive;

@TeleOp(name = "Basic: Custom Holonomic Drive", group = "Linear OpMode")
public class CustomHolonomicDrive extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

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
        HANGING_ARMS_OUT,
        PULL_UP_ROBOT,
    }

    private State state = State.IDLE;

    @Override
    public void runOpMode() {
        SimpleDrive drive = new SimpleDrive(this);
        drive.start();

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
        viperSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        leftHangingMotor.setTargetPosition(0);
        leftHangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftHangingMotor.setPower(1);
        leftHangingMotor.setDirection(DcMotor.Direction.REVERSE);
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
        double safeTime = 0;
        double hangingTime = 0;

        // Declare initial positions for parts
        double extendServoPosition = 0.0;
        double bucketServoPosition = 0.0;
        double intakeServoPosition = 0.0;
        double sampleClawServoPosition = 0.0;
        double wristServoPosition = 0.0;
        double specimenClawServoPosition = 0.0;

        double leftHangingServoPosition = 0.0;
        double rightHangingServoPosition = 0.0;

        int viperSlideMotorPosition = 0;

        int hangingMotorPosition = 0;

        // Declare positions for parts to move to
        int liftDown = 0;
        int liftTopBucket = 6180;
        int liftBottomBucket = 3480;
        int liftTopBar = 2890;
        int liftBottomBar = 960;

        double bucketDrop = 0.37;
        double bucketLoad = 0.5;

        int hangingMotorIn = 0;
        int hangingMotorOut = 1552;

        double leftHangingServoUp = 1.0;
        double rightHangingServoUp = 0.0;
        double leftHangingServoForward = 0.5;
        double rightHangingServoForward = 0.5;

        double extendClosed = 0.0;
        double extendExtended = 1.0;

        double intakeDown = 0.15;
        double intakeUp = 1;
        double intakeOutOfWay = 0.3;

        double wristLoad = 0;
        double wristDrop = 1;
        double wristOutOfWay = 0.2;

        double sampleClawClosed = 0.5;
        double sampleClawOpen = 0;

        double specimenClawClosed = 0;
        double specimenClawOpen = 0.5;

        // Declare buttons for switching between states
        boolean aPrev = false;
        boolean bPrev = false;
        boolean xPrev = false;
        boolean yPrev = false;
        boolean rbPrev = false;

        boolean dpadUpPrev = false;

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

            switch (state) {
                case IDLE:
                    if (runtime.seconds() > safeTime + 2.5) {
                        viperSlideMotorPosition = liftDown;
                        bucketServoPosition = bucketLoad;
                        extendServoPosition = extendClosed;
                        wristServoPosition = wristDrop;
                        sampleClawServoPosition = sampleClawClosed;
                        specimenClawServoPosition = specimenClawClosed;
                        leftHangingServoPosition = leftHangingServoUp;
                        rightHangingServoPosition = rightHangingServoUp;
                        hangingMotorPosition = hangingMotorIn;
                    }
                    intakeServoPosition = intakeUp;


                    if (a) {
                        state = State.EXTENDED;
                        wristTime = runtime.seconds();
                    } else if (y) {
                        state = State.PLACE_SPECIMEN_HIGH_BAR;
                    } else if (x) {
                        state = State.PLACE_SPECIMEN_LOW_BAR;
                    } else if (dpadUp) {
                        state = State.HANGING_ARMS_OUT;
                        hangingTime = runtime.seconds();
                    }
                    break;
                case EXTENDED:
                    if (runtime.seconds() > wristTime + 0.5) {
                        viperSlideMotorPosition = liftDown;
                        bucketServoPosition = bucketLoad;
                        extendServoPosition = extendExtended;
                        intakeServoPosition = intakeDown;
                        wristServoPosition = wristLoad;
                        sampleClawServoPosition = sampleClawOpen;
                        specimenClawServoPosition = specimenClawClosed;
                        leftHangingServoPosition = leftHangingServoUp;
                        rightHangingServoPosition = rightHangingServoUp;
                        hangingMotorPosition = hangingMotorIn;

                    }
                        wristServoPosition = wristLoad;
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
                    leftHangingServoPosition = leftHangingServoUp;
                    rightHangingServoPosition = rightHangingServoUp;
                    hangingMotorPosition = hangingMotorIn;

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
                    leftHangingServoPosition = leftHangingServoUp;
                    rightHangingServoPosition = rightHangingServoUp;
                    hangingMotorPosition = hangingMotorIn;

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
                    leftHangingServoPosition = leftHangingServoUp;
                    rightHangingServoPosition = rightHangingServoUp;
                    hangingMotorPosition = hangingMotorIn;

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
                    leftHangingServoPosition = leftHangingServoUp;
                    rightHangingServoPosition = rightHangingServoUp;
                    hangingMotorPosition = hangingMotorIn;

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
                    intakeServoPosition = intakeOutOfWay;

                    if (runtime.seconds() > liftedTime + 1) {
                        viperSlideMotorPosition = liftTopBucket;
                    }

                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    specimenClawServoPosition = specimenClawClosed;
                    leftHangingServoPosition = leftHangingServoUp;
                    rightHangingServoPosition = rightHangingServoUp;
                    hangingMotorPosition = hangingMotorIn;

                    if (runtime.seconds() > liftedTime + 0.5) {
                        wristServoPosition = wristOutOfWay;
                    }
                    sampleClawServoPosition = sampleClawOpen;

                    if (a) {
                        state = State.DROP_HIGH_BUCKET;
                    } else if (b) {
                        state = State.IDLE;
                    }
                    break;
                case LIFTED_LOW_BUCKET:
                    intakeServoPosition = intakeOutOfWay;

                    if (runtime.seconds() > liftedTime + 1) {
                        viperSlideMotorPosition = liftBottomBucket;
                    }
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    specimenClawServoPosition = specimenClawClosed;
                    leftHangingServoPosition = leftHangingServoUp;
                    rightHangingServoPosition = rightHangingServoUp;
                    hangingMotorPosition = hangingMotorIn;

                    if (runtime.seconds() > liftedTime + 0.5) {
                        wristServoPosition = wristOutOfWay;
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
                    intakeServoPosition = intakeOutOfWay;
                    wristServoPosition = wristOutOfWay;
                    sampleClawServoPosition = sampleClawOpen;
                    specimenClawServoPosition = specimenClawClosed;
                    leftHangingServoPosition = leftHangingServoUp;
                    rightHangingServoPosition = rightHangingServoUp;
                    hangingMotorPosition = hangingMotorIn;

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
                    intakeServoPosition = intakeOutOfWay;
                    wristServoPosition = wristOutOfWay;
                    sampleClawServoPosition = sampleClawOpen;
                    specimenClawServoPosition = specimenClawClosed;
                    leftHangingServoPosition = leftHangingServoUp;
                    rightHangingServoPosition = rightHangingServoUp;
                    hangingMotorPosition = hangingMotorIn;

                    if (a) {
                        state = State.IDLE;
                    }
                    if (b) {
                        state = State.LIFTED_LOW_BUCKET;
                    }
                    break;
                case HANGING_ARMS_OUT:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketDrop;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeDown;
                    wristServoPosition = wristLoad;
                    sampleClawServoPosition = sampleClawClosed;
                    specimenClawServoPosition = specimenClawClosed;
                    leftHangingServoPosition = leftHangingServoForward;
                    rightHangingServoPosition = rightHangingServoForward;

                    if (runtime.seconds() > hangingTime + 1) {
                        hangingMotorPosition = hangingMotorOut;
                    }

                    if (a) {
                        state = State.PULL_UP_ROBOT;
                        hangingTime = runtime.seconds();
                    } else if (b) {
                        state = State.IDLE;
                    }
                    break;
                case PULL_UP_ROBOT:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketDrop;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeDown;
                    wristServoPosition = wristLoad;
                    sampleClawServoPosition = sampleClawClosed;
                    specimenClawServoPosition = specimenClawClosed;
                    hangingMotorPosition = hangingMotorIn;

                    if (runtime.seconds() > hangingTime + 0.75) {
                        leftHangingServoPosition = leftHangingServoUp;
                        rightHangingServoPosition = rightHangingServoUp;
                    }

                    if (b) {
                        state = State.HANGING_ARMS_OUT;
                    }
                    break;
            }

            // Set servo positions
            extendServo.setPosition(extendServoPosition);
            bucketServo.setPosition(bucketServoPosition);
            intakeServo.setPosition(intakeServoPosition);
            sampleClawServo.setPosition(sampleClawServoPosition);
            wristServo.setPosition(wristServoPosition);
            specimenClawServo.setPosition(specimenClawServoPosition);
            leftHangingServo.setPosition(leftHangingServoPosition);
            rightHangingServo.setPosition(rightHangingServoPosition);

            // Set (non-drive) motor power
            viperSlideMotor.setTargetPosition(viperSlideMotorPosition);
            if (viperSlideMotor.isBusy()) {
                viperSlideMotor.setPower(1);
            } else if (!viperSlideMotor.isBusy() && viperSlideMotorPosition == 0) {
                viperSlideMotor.setPower(0);
            }
            leftHangingMotor.setTargetPosition(hangingMotorPosition);
            if (leftHangingMotor.isBusy()) {
                leftHangingMotor.setPower(1);
            } else if (!leftHangingMotor.isBusy() && hangingMotorPosition == 0) {
                leftHangingMotor.setPower(0);
            }
            rightHangingMotor.setTargetPosition(hangingMotorPosition);
            if (rightHangingMotor.isBusy()) {
                rightHangingMotor.setPower(1);
            } else if (!rightHangingMotor.isBusy() && hangingMotorPosition == 0) {
                rightHangingMotor.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("State", state);
            telemetry.addData("extendServo position", extendServoPosition);
            telemetry.addData("bucketServo position", bucketServoPosition);
            telemetry.addData("intakeServo position", intakeServoPosition);
            telemetry.addData("clawServo position", sampleClawServoPosition);
            telemetry.addData("clawWristServo position", wristServoPosition);
            telemetry.addData("specimenClawServo position", specimenClawServoPosition);
            telemetry.addData("leftHangingMotor position", leftHangingServoPosition);
            telemetry.addData("rightHangingServo position", rightHangingServoPosition);
            telemetry.addData("hangingMotorPosition", hangingMotorPosition);
            telemetry.addData("Viper encoder", viperSlideMotor.getCurrentPosition());
            telemetry.addData("Left hanging motor encoder", leftHangingMotor.getCurrentPosition());
            telemetry.addData("Right hanging motor encoder", rightHangingMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
