package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SimpleDrive extends Thread{

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    LinearOpMode opMode;

    public SimpleDrive(LinearOpMode opMode) {
        this.opMode = opMode;
        this.setName("Drive");
        init();
    }

    private void init(){

        leftFrontDrive = opMode.hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = opMode.hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = opMode.hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = opMode.hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void run() {
        super.run();

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {


            double axial = -opMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = opMode.gamepad1.left_stick_x;
            double yaw = opMode.gamepad1.right_stick_x;

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
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // If right trigger pressed enter half speed
            if (opMode.gamepad1.right_trigger > 0) {
                leftFrontPower /= 2;
                rightFrontPower /= 2;
                leftBackPower /= 2;
                rightBackPower /= 2;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }
    }
}
