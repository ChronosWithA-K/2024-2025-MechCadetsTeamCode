package test.code;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

import common.Config;
import common.Drive;
import common.Logger;

@TeleOp(name="DecelerationTest", group="Test")
@SuppressLint("DefaultLocale")

public class DecelerationTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public static double MIN_POWER = 0.25;
    public static double MAX_POWER = 0.95;
    public static double INCHES_TO_DRIVE = 60;

    double ODOMETER_WHEEL_DIAMETER = 48/25.4;   // wheel diameter in inches
    double ODOMETER_COUNT_PER_REV = 2000;       // encoder count
    double ODOMETER_COUNT_PER_WHEEL_REV = ODOMETER_COUNT_PER_REV * ODOMETER_WHEEL_DIAMETER * Math.PI;

    Drive drive  = null;

    @Override
    public void runOpMode() {
        try {
            drive = new Drive(this);
            drive.start();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            forwardTest();

        } catch (Exception e) {
            Logger.error(e, "Error");
        }

    }

    private void forwardTest() {

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        while (opModeIsActive()) {

            for (double power = MIN_POWER; power <= MAX_POWER; power += 0.05) {

                // move forward to calculate the overshoot error
                double maxVelocity = move(power, 1, 0);
                sleep(2000);

                double odometerInches = Math.abs(drive.odometer.getCurrentPosition()) / ODOMETER_COUNT_PER_WHEEL_REV;
                double error = odometerInches - INCHES_TO_DRIVE;
                displayResults(power, voltageSensor.getVoltage(), maxVelocity, error);

                // return to start position
                move(power, -1, 0);
                sleep(2000);

                // test adjusting for overshoot error
                move(power, 1, error);
                sleep(2000);

                odometerInches = Math.abs(drive.odometer.getCurrentPosition()) / ODOMETER_COUNT_PER_WHEEL_REV;
                error = odometerInches - INCHES_TO_DRIVE;
                displayResults(power, voltageSensor.getVoltage(), maxVelocity, error);

                // return to start position
                move(power, -1, error);
                sleep(2000);

                Logger.addLine("\n");
            }
        }
    }

    private double move (double power, int sign, double error) {

        // Determine new target position
        int target = (int) (Math.max(INCHES_TO_DRIVE - error, 0) * drive.encoderTicksPerInch()) * sign;

        for (DcMotor motor : drive.motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(target);
        }

        // Drive until all motor reach target position
        double maxVelocity = 0;
        drive.accelerationReset();
        while (opModeIsActive()) {
            for (DcMotor motor : drive.motors) {
                double rampPower = drive.accelerationLimit(power);
                motor.setPower(rampPower * sign);
            }

            boolean busy = false;
            for (DcMotor motor : drive.motors) {
                busy |= motor.isBusy();
            }
            if (! busy) break;

            double velocity = drive.odometer.getVelocity();
            maxVelocity = Math.max(maxVelocity, velocity);
        }

        // Stop all motion;
        for (DcMotor motor : drive.motors)
            motor.setPower(0);

        return maxVelocity;
    }

    private void displayResults(double power, double voltage, double maxVelocity, double error) {
        Logger.message("%s",
                String.format("power %4.1f   ", power) +
                String.format("voltage: %5.2f", voltage) +
                String.format("max velocity:  %5.0f   ", maxVelocity) +
                String.format("error %6.2f", error));
    }
}
