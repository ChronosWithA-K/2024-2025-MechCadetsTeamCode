package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import common.SimpleDrive;

@TeleOp(name="Basic: Custom Holonomic Drive", group="Linear OpMode")
public class CustomHolonomicDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor viperSlideMotor = null;

//    private DcMotor xEncoder = null; // If uncommented and not attached to the robot, things will break
//    private DcMotor yEncoder = null;

    private Servo extendServo = null;
    private Servo bucketServo = null;
    private Servo intakeServo = null;
    private Servo clawServo = null;
    private Servo clawWristServo = null;

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
    }
    private State state = State.IDLE;

    @Override
    public void runOpMode() {

        SimpleDrive drive = new SimpleDrive(this);
        drive.start();

//        xEncoder = hardwareMap.get(DcMotor.class, "x_encoder");
//        yEncoder = hardwareMap.get(DcMotor.class, "y_encoder");

        extendServo = hardwareMap.get(Servo.class, "extend_servo");
        bucketServo = hardwareMap.get(Servo.class, "bucket_servo");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawWristServo = hardwareMap.get(Servo.class,  "claw_wrist_servo");

        viperSlideMotor = hardwareMap.get(DcMotor.class, "viper_slide_motor");

        viperSlideMotor.setTargetPosition(0);
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideMotor.setPower(1);

        viperSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double liftedTime = 0;

        double extendServoPosition = 0.0;
        double bucketServoPosition = 0.0;
        double intakeServoPosition = 0.0;
        double clawServoPosition = 0.0;
        double clawWristServoPosition = 0.0;

        int viperSlideMotorPosition = 0;

        int liftDown = 0;
        int liftTopBucket = 3260; // change
        int liftBottomBucket = 1730; // change
        int liftTopBar = 1630; // change
        int liftBottomBar = 580; // change

        double bucketDrop = 0.37;
        double bucketLoad = 0.5;

        double extendClosed = 0;
        double extendExtended = 1;

        double intakeDown = 0.175;
        double intakeUp = 1;

        double wristLoad = 0.5;
        double wristDrop = 1;
        double wristLift = 0.2;

        double clawClosed = 0;
        double clawOpen = 0.5;

        boolean aPrev = false;
        boolean bPrev = false;
        boolean xPrev = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            boolean a = gamepad1.a && !aPrev;
            aPrev = gamepad1.a;
            boolean b = gamepad1.b && !bPrev;
            bPrev = gamepad1.b;
            boolean x = gamepad1.x && !xPrev;
            xPrev = gamepad1.x;

            switch (state) {
                case IDLE:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristLift;
                    clawServoPosition = clawClosed;

                    if (a) {
                        state = State.EXTENDED;
                    }
                    if (b) {
                        state = State.PLACE_SPECIMEN_HIGH_BAR;
                    }
                    if (x) {
                        state = State.PLACE_SPECIMEN_LOW_BAR;
                    }
                    break;
                case EXTENDED:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeDown;
                    clawWristServoPosition = wristLoad;
                    clawServoPosition = clawOpen;

                    if (a) {
                        state = State.IDLE;
                    } else if (b) {
                        state = State.GRABBED;
                    }
                    break;
                case PLACE_SPECIMEN_HIGH_BAR:
                    viperSlideMotorPosition = liftTopBar;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristLoad;
                    clawServoPosition = clawClosed;

                    if (b) {
                        state = State.IDLE;
                    }
                    break;
                case PLACE_SPECIMEN_LOW_BAR:
                    viperSlideMotorPosition = liftBottomBar;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristLoad;
                    clawServoPosition = clawClosed;

                    if (x) {
                        state = State.IDLE;
                    }
                    break;
                case GRABBED:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeDown;
                    clawWristServoPosition = wristLoad;
                    clawServoPosition = clawClosed;

                    if (a) {
                        state = State.LOADED;
                    } else if (b) {
                        state = State.EXTENDED;
                    }
                    break;
                case LOADED:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristDrop;
                    clawServoPosition = clawClosed;

                    if (a) {
                        state = State.LIFTED_HIGH_BUCKET;
                        liftedTime = runtime.seconds();
                    } else if (b) {
                        state = State.EXTENDED;
                    } else if (x) {
                        state = State.LIFTED_LOW_BUCKET;
                    }
                    break;
                case LIFTED_HIGH_BUCKET:
                    if (runtime.seconds() > liftedTime + 1) {
                        viperSlideMotorPosition = liftTopBucket;
                    }
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;

                    if (runtime.seconds() > liftedTime + 0.5){
                        clawWristServoPosition = wristLift;
                    }
                    clawServoPosition = clawOpen;

                    if (a) {
                        state = State.DROP_HIGH_BUCKET;
                    }
                    break;
                case LIFTED_LOW_BUCKET:
                    if (runtime.seconds() > liftedTime + 1) {
                        viperSlideMotorPosition = liftBottomBucket;
                    }
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;

                    if (runtime.seconds() > liftedTime + 0.5){
                        clawWristServoPosition = wristLift;
                    }
                    clawServoPosition = clawOpen;

                    if (a) {
                        state = State.DROP_LOW_BUCKET;
                    }
                    break;
                case DROP_HIGH_BUCKET:
                    viperSlideMotorPosition = liftTopBucket;
                    bucketServoPosition = bucketDrop;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristLift;
                    clawServoPosition = clawOpen;

                    if (a) {
                        state = State.IDLE;
                    }
                    break;
                case DROP_LOW_BUCKET:
                    viperSlideMotorPosition = liftBottomBucket;
                    bucketServoPosition = bucketDrop;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristLift;
                    clawServoPosition = clawOpen;

                    if (a) {
                        state = State.IDLE;
                    }
            }

            // Set servo positions
            extendServo.setPosition(extendServoPosition);
            bucketServo.setPosition(bucketServoPosition);
            intakeServo.setPosition(intakeServoPosition);
            clawServo.setPosition(clawServoPosition);
            clawWristServo.setPosition(clawWristServoPosition);

             // Set (non-drive) motor power
            viperSlideMotor.setTargetPosition(viperSlideMotorPosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("xEncoder", "%4.2f, %4.2f", xEncoder);
//            telemetry.addData("yEncoder", "%4.2f, %4.2f", yEncoder);
            telemetry.addData("State", state);
            telemetry.addData("extendServo position: ", extendServoPosition);
            telemetry.addData("bucketServo position: ", bucketServoPosition);
            telemetry.addData("intakeServo position: ", intakeServoPosition);
            telemetry.addData("clawServo position: ", clawServoPosition);
            telemetry.addData("clawWristServo position: ", clawWristServoPosition);
            telemetry.addData("Viper encoder: ", viperSlideMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
