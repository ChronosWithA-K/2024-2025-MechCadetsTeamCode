package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Test")
public class Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private IMU imu = null;

    private Limelight3A limelight;

    private Follower follower;

    private Pose startPose = new Pose(10.000, 58.000, Point.CARTESIAN);

    private int pathIndex = 0;
    private ArrayList<PathChain> pathChains = new ArrayList<PathChain>();

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor viperSlideMotor = null;

    private Servo extendServo = null;
    private Servo bucketServo = null;
    private Servo intakeServo = null;
    private Servo sampleClawServo = null;
    private Servo wristServo = null;
    private Servo specimenClawServo = null;

    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        viperSlideMotor = hardwareMap.get(DcMotor.class, "viper_slide_motor");

        imu = hardwareMap.get(IMU.class, "imu");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        /*
         * Pipeline 0 is for the red sample
         * 1 is for the blue sample
         * 2 is for the yellow sample
         * 3 is for april tag 11
         * 4 is for april tag 12
         * 5 is for april tag 13
         * 6 is for april tag 14
         * 7 is for april tag 15
         * 8 is for april tag 16
         * 9 for all at once
         */

        extendServo = hardwareMap.get(Servo.class, "extend_servo");
        bucketServo = hardwareMap.get(Servo.class, "bucket_servo");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        sampleClawServo = hardwareMap.get(Servo.class, "sample_claw_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        specimenClawServo = hardwareMap.get(Servo.class, "specimen_claw_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        viperSlideMotor.setTargetPosition(0);
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideMotor.setPower(1);
        viperSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        // Start the limelight polling for data (getLatestResult() will return null without this)
        limelight.start();
        limelight.pipelineSwitch(9);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        telemetry.update();
        pathChains.add(follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Point(startPose),
                                        new Point(35.000, 70.000, Point.CARTESIAN) // Drive to chamber
                                )
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build());
        pathChains.add(follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Point(35.000, 70.000, Point.CARTESIAN),
                                        new Point(13.000, 19.000, Point.CARTESIAN) // Drive to closest sample
                                )
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build());
        pathChains.add(follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Point(13.000, 19.000, Point.CARTESIAN),
                                        new Point(19.995, 124.173, Point.CARTESIAN) // Drive to bucket
                                )
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build());
        pathChains.add(follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Point(19.995, 124.173, Point.CARTESIAN),
                                        new Point(10.922, 7.561, Point.CARTESIAN) // Park
                                )
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build());
        pathIndex = 0;
        follower.followPath(pathChains.get(pathIndex));
        waitForStart();
        runtime.reset();

        // Declare initial positions for parts
        double extendServoPosition = 0.0;
        double bucketServoPosition = 0.0;
        double intakeServoPosition = 0.0;
        double sampleClawServoPosition = 0.0;
        double wristServoPosition = 0.0;
        double specimenClawServoPosition = 0.0;

        int viperSlideMotorPosition = 0;

        // Declare positions for parts to move to
        int liftDown = 0;
        int liftTopBucket = 6180;
        int liftBottomBucket = 3480;
        int liftTopBar = 2890;
        int liftBottomBar = 960;

        double bucketDrop = 0.37;
        double bucketLoad = 0.5;

        double extendClosed = 0.0;
        double extendExtended = 1.0;

        double intakeDown = 0.26;
        double intakeUp = 1;
        double intakeIdle = 0.65;

        double wristLoad = 0.5;
        double wristDrop = 1;
        double wristLift = 0.2;

        double sampleClawClosed = 0;
        double sampleClawOpen = 0.4;

        double specimenClawClosed = 0;
        double specimenClawOpen = 0.5;

        LLStatus limelightStatus = limelight.getStatus();
        telemetry.addData("Pipeline", "Index: %d, Type: %s", limelightStatus.getPipelineIndex(), limelightStatus.getPipelineType());

        LLResult limelightResult = limelight.getLatestResult();

        if (limelightResult != null) {
            limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
            // Access general information
            Pose3D botpose = limelightResult.getBotpose();

            double captureLatency = limelightResult.getCaptureLatency();
            double targetingLatency = limelightResult.getTargetingLatency();
            double parseLatency = limelightResult.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(limelightResult.getPythonOutput()));

            if (limelightResult.isValid()) {
                double targetX = limelightResult.getTx(); // How far left or right the target is (degrees)
                double targetY = limelightResult.getTy(); // How far up or down the target is (degrees)
                double targetArea = limelightResult.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", targetX);
                telemetry.addData("Target Y", targetY);
                telemetry.addData("Target Area", targetArea);

                if (botpose != null) {
                    double robotX = botpose.getPosition().x;
                    double robotY = botpose.getPosition().y;
                    double robotRotation = botpose.getOrientation().getYaw();
                    telemetry.addData("MT1 Location", "(" + robotX + ", " + robotY + ", " + robotRotation + ")");
                }

                telemetry.addData("Botpose", botpose.toString());

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = limelightResult.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = limelightResult.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            } else {
                telemetry.addData("Limelight", "No targets");
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }

        while(opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()){
                if (pathIndex < pathChains.size() - 1){
                    pathIndex++;
                    follower.followPath(pathChains.get(pathIndex));
                }
            }
            switch (pathIndex){
                case 0:

                    break;
                case 1:

                    break;
                case 2:

                    break;
                case 3:

                    break;
            }
            telemetry.update();
        }
    }
}
