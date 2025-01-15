package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AyanshAuto", group = "Test")
public class AutoNew extends LinearOpMode {

    private IMU imu = null;

    private Limelight3A limelight;

    private Follower follower;
    // uses global coordinate system
    private Pose startPose = new Pose(63, 24);
    private Pose preHangSpecimenPose = new Pose(35, 0);
    private Pose preHangSpecimenPose2 = new Pose(35, -2);
    private Pose preHangSpecimenPose3 = new Pose(35, -3);


    private Pose hangSpecimenPose = new Pose(31.39, 0);
    private Pose hangSpecimen2Pose = new Pose(31.39, -2);
    private Pose hangSpecimen3Pose = new Pose(31.39, -3);


    private Pose prePickUpSpecimenPosept1 = new Pose(55, 0, 0);
    private Pose prePickUpSpecimenPosept2 = new Pose(55, 48, Math.PI / 2);
    private Pose pickUpSpecimenPose = new Pose(63, 48, Math.PI);

    private Pose pushblockpt1 = new Pose(57, 36);
    private Pose pushblockpt2 = new Pose(12, 36);
    private Pose pushblockpt3 = new Pose(12, 55, Math.PI / 2);
    private Pose pushblockdone = new Pose(57, 55, Math.PI / 2);
    private Pose retreat = new Pose(40, 55, 0);


    private int pathIndex = 0;
    private ArrayList<PathChain> pathChains = new ArrayList<PathChain>();

    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor viperSlideMotor = null;


    private Servo bucketServo = null;
    private Servo intakeServo = null;

    private Servo wristServo = null;
    private Servo specimenClawServo = null;

    public static double hangDelay = 0.5;
    public static double hangDelay2 = 0.5;


    private void addLine(Pose start, Pose end) {
        pathChains.add(follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Point(start),
                                        new Point(end)

                                )
                        )
                )
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build());
    }


    @Override
    public void runOpMode() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.5);

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

        bucketServo = hardwareMap.get(Servo.class, "bucket_servo");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        specimenClawServo = hardwareMap.get(Servo.class, "specimen_claw_servo");

        viperSlideMotor.setTargetPosition(0);
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideMotor.setPower(1);
        viperSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        // Start the limelight polling for data (getLatestResult() will return null without this)
        limelight.start();
        limelight.pipelineSwitch(9);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.update();
        addLine(startPose, preHangSpecimenPose); // origin to pre   case1
        addLine(preHangSpecimenPose, hangSpecimenPose); // pre to hang case2
        addLine(hangSpecimenPose, prePickUpSpecimenPosept1); // hang to pre1 case3
        addLine(prePickUpSpecimenPosept1, prePickUpSpecimenPosept2); // pre1 to pre2 case4
        addLine(prePickUpSpecimenPosept2, pickUpSpecimenPose); // pre to pick case5
        addLine(pickUpSpecimenPose, preHangSpecimenPose2); // pick to  pre case6
        addLine(preHangSpecimenPose2, hangSpecimen2Pose); // pre to hang2 case7
//        addLine(hangSpecimen2Pose,pushblockpt1); // hang to pt1 case8
//        addLine(pushblockpt1, pushblockpt2); // pt1 to pt2 case9
//        addLine(pushblockpt2, pushblockpt3); // pt2 to pt3 case10
//        addLine(pushblockpt3,pushblockdone); // pt3 to deliver case11
//        addLine(pushblockdone,retreat); // deliver to retreat case12
//        addLine(retreat,pickUpSpecimenPose); // retreat to pick case13
//        addLine(pickUpSpecimenPose,preHangSpecimenPose3); // pick to pre case14
//        addLine(preHangSpecimenPose3,hangSpecimen3Pose); // pre to hang3 case15


        pathIndex = 0;
        follower.followPath(pathChains.get(pathIndex));
        waitForStart();
        runtime.reset();

        // Declare positions for parts to move to
        int liftTopBucket = 6180;
        int liftTopBar = 2890;
        int engaged = 1840;


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
        double currentStageStartTime = runtime.seconds();
        while (opModeIsActive()) {
            follower.update();
            if (follower.getCurrentPath() != null && follower.atParametricEnd()) {
                if (pathIndex < pathChains.size() - 1) {
                    pathIndex++;
                    currentStageStartTime = runtime.seconds();
                    follower.resetCurrentPath();
                }
            }
            telemetry.addData("currentStageStartTime : ", currentStageStartTime);
            telemetry.addData("runtime.seconds() : ", runtime.seconds());
            switch (pathIndex) {
                case 0:
                    specimenClawServo.setPosition(specimenClawClosed);
                    viperSlideMotor.setTargetPosition(liftTopBar);
                    bucketServo.setPosition(liftTopBucket);
                    intakeServo.setPosition(0.6);
                    wristServo.setPosition(0.3);
                    telemetry.addLine("Stage Initiation Finished");
                    if (follower.getCurrentPath() == null) {
                        follower.followPath(pathChains.get(pathIndex));
                    }
                    break;
                case 1:
                    if (follower.getCurrentPath() == null) {
                        follower.followPath(pathChains.get(pathIndex));
                    }
                    break;
                case 2:
                    if (currentStageStartTime > runtime.seconds() - hangDelay) {
                        viperSlideMotor.setTargetPosition(engaged);
                        telemetry.addLine("Stage Prep finished");
                        telemetry.addData("runtime.seconds() + hangDelay", runtime.seconds() - hangDelay);
                    } else {
                        specimenClawServo.setPosition(specimenClawOpen);
                        viperSlideMotor.setTargetPosition(0);
                        telemetry.addLine("Stage Hang");
                        if (follower.getCurrentPath() == null) {
                            follower.followPath(pathChains.get(pathIndex));
                        }
                    }
                    break;
                case 3:
                    specimenClawServo.setPosition(specimenClawOpen);
                    if (follower.getCurrentPath() == null) {
                        follower.followPath(pathChains.get(pathIndex));
                    }
                    telemetry.addLine("Stage Pre 1");

                    break;
                case 4:
                    specimenClawServo.setPosition(specimenClawOpen);
                    if (follower.getCurrentPath() == null) {
                        follower.followPath(pathChains.get(pathIndex));
                    }
                    telemetry.addLine("Stage Pre 2");

                    break;

                case 5:
                    if (currentStageStartTime > runtime.seconds() - hangDelay2) {
                        specimenClawServo.setPosition(specimenClawClosed);
                    } else {
                        viperSlideMotor.setTargetPosition(liftTopBar);
                        if (follower.getCurrentPath() == null) {
                            follower.followPath(pathChains.get(pathIndex));
                        }
                        telemetry.addLine("Stage Pick finished");

                    }
                    break;
                case 6:
                    if (currentStageStartTime > runtime.seconds() - hangDelay2) {
                        specimenClawServo.setPosition(specimenClawClosed);
                    }
                    telemetry.addLine("Stage 5 finished");
                    if (follower.getCurrentPath() == null) {
                        follower.followPath(pathChains.get(pathIndex));
                    }

                    break;
                case 7:
                    if (currentStageStartTime < runtime.seconds() + hangDelay) {
                        viperSlideMotor.setTargetPosition(engaged);
                        telemetry.addLine("Stage Hang2 finished");
                    } else {
                        wristServo.setPosition(1);
                        if (follower.getCurrentPath() == null) {
                            follower.followPath(pathChains.get(pathIndex));
                        }
                    }
                    break;
//                case 8:
//                    viperSlideMotor.setTargetPosition(0);
//                    if (follower.getCurrentPath() == null) {
//                        follower.followPath(pathChains.get(pathIndex));
//                    }
//                    telemetry.addLine("Stage Pre block 1");
//                    break;
//                case 9:
//                    viperSlideMotor.setTargetPosition(0);
//                    if (follower.getCurrentPath() == null) {
//                        follower.followPath(pathChains.get(pathIndex));
//                    }
//                    telemetry.addLine("Stage Pre block 2");
//
//                    break;
//                case 10:
//                    viperSlideMotor.setTargetPosition(0);
//                    if (follower.getCurrentPath() == null) {
//                        follower.followPath(pathChains.get(pathIndex));
//                    }
//                    telemetry.addLine("Stage Pre block 3");
//
//                    break;
//                case 11:
//                    viperSlideMotor.setTargetPosition(0);
//                    if (follower.getCurrentPath() == null) {
//                        follower.followPath(pathChains.get(pathIndex));
//                    }
//                    telemetry.addLine("Stage block delivered");
//
//                    break;
//                case 12:
//                    viperSlideMotor.setTargetPosition(0);
//                    if (follower.getCurrentPath() == null) {
//                        follower.followPath(pathChains.get(pathIndex));
//                    }
//                    telemetry.addLine("Stage retreat");
//
//                    break;
//                case 13:
//                    if (currentStageStartTime > runtime.seconds() - hangDelay2 * 2) {
//                        specimenClawServo.setPosition(specimenClawClosed);
//                    } else {
//                        viperSlideMotor.setTargetPosition(liftTopBar);
//                        if (follower.getCurrentPath() == null) {
//                            follower.followPath(pathChains.get(pathIndex));
//                        }
//                        telemetry.addLine("Stage Pick finished");
//
//                    }
//                    break;
//                case 14:
//                    specimenClawServo.setPosition(specimenClawClosed);
//                    if (follower.getCurrentPath() == null) {
//                        follower.followPath(pathChains.get(pathIndex));
//                    }
//                    telemetry.addLine("Stage Pre ");
//
//                    break;
//                case 15:
//                    if (currentStageStartTime < runtime.seconds() + hangDelay) {
//                        viperSlideMotor.setTargetPosition(engaged);
//                        telemetry.addLine("Stage Hang3 finished");
//                    } else if (currentStageStartTime < runtime.seconds() + hangDelay) {
//                        specimenClawServo.setPosition(specimenClawOpen);
//                        viperSlideMotor.setTargetPosition(0);
//
//                    } else if (currentStageStartTime < runtime.seconds() + hangDelay * 2) {
//                        intakeServo.setPosition(1);
//                    } else {
//                        wristServo.setPosition(1);
//                        if (follower.getCurrentPath() == null) {
//                            follower.followPath(pathChains.get(pathIndex));
//                        }
//                    }
//                    break;
//
            }
            telemetry.update();
        }
    }
}
