/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package test.code;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.Drive;
import common.Gyro;
import common.Logger;
import utils.Increment;

@TeleOp(name="Gyro Test", group="Test")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class GyroTest extends LinearOpMode {

    final static double TURN_SPEED      = 0.25;
    final double TURN_MIN_SPEED         = 0.2;
    final double TURN_MAX_SPEED         = 0.9;

    final double TURN_RAMP_UP_TIME      = 1000;                       // turn ramp up time in milliseconds
    final double TURN_RAMP_DOWN_DEGREES = 5;

    public static double speed = TURN_SPEED;
    private double heading;

    private Telemetry.Item speedMsg;

    private Drive drive;
    private AHRS gyro;

    Gyro gyro1;
    Gyro gyro2;
    Gyro gyro3;

    @Override
    public void runOpMode() {

        try {
            run();
        } catch (Exception e)  {
            Logger.error(e, "Exception");
        }
    }

    private void run() {

        gyro1 = new Gyro(hardwareMap, "navx");
        gyro2 = new Gyro(hardwareMap, "imu");
        gyro3 = new Gyro(hardwareMap, "imuExpansion",
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        drive = new Drive(this);
        initNavx();
        heading = 0;

        Increment speedIncrement = new Increment(0.01, 0.05, 0.1);

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.clear();

        speedMsg =  telemetry.addData("Speed", "%4.2f", speed);
        Telemetry.Item compassMsg =  telemetry.addData("compass heading", 0);
        Telemetry.Item yawNavxMsg =  telemetry.addData("yaw navx", 0);
        Telemetry.Item yawImuMsg =  telemetry.addData("yaw imu", 0);

        telemetry.addData("\nControls", "\r\n" +
                "  dpad up - increase speed\n" +
                "  dpad down - decrease speed\n" +
                "  y - turn north\n" +
                "  a - turn east\n" +
                "  x - turn south\n" +
                "  b - turn west\n" +
                "\n");

        while (opModeIsActive()) {
            if (gamepad1.y) {
                heading = 0;
                turnTo(heading);
            }

            else if (gamepad1.b) {
                heading = 90;
                turnTo(heading);
            }

            else if (gamepad1.a) {
                heading = 180;
                turnTo(heading);
            }

            else if (gamepad1.x) {
                heading = 270;
                turnTo(heading);
            }

            else if (gamepad1.dpad_up) {
                // increase the speed to travel
                speedIncrement.reset();
                while (gamepad1.dpad_up) {
                    speed = Math.min(speed + speedIncrement.get(), TURN_MAX_SPEED);
                    setDisplaySpeed();
                    telemetry.update();
                }

            } else if (gamepad1.dpad_down) {
                // decrease the speed the travel
                speedIncrement.reset();
                while (gamepad1.dpad_down) {
                    speed = Math.max(speed - speedIncrement.get(), TURN_MIN_SPEED);
                    setDisplaySpeed();
                    telemetry.update();
                }
            }

            setDisplaySpeed();
            compassMsg.setValue("%f", gyro.getCompassHeading());
            yawNavxMsg.setValue("%6.2f (error: %6.2f)", gyro1.getYaw(), heading - gyro1.getYaw());
            yawImuMsg.setValue("%6.2f (error: %6.2f)", gyro2.getYaw(), heading - gyro2.getYaw());
            telemetry.update();

            displayDashboardTelemetry();

        }
    }

    private void displayDashboardTelemetry() {

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        dashboardTelemetry.addLine("  ");
        dashboardTelemetry.addLine("Controls");
        dashboardTelemetry.addLine("  a - turn east");
        dashboardTelemetry.addLine("  b - turn west");
        dashboardTelemetry.addLine("  x - turn south");
        dashboardTelemetry.addLine("  y - turn north");

        dashboardTelemetry.addData("1. speed", "%4.2f", speed);
        dashboardTelemetry.addData("3. yaw navx", "%6.2f (error: %6.2f)", gyro1.getYaw(), heading - gyro1.getYaw());
        dashboardTelemetry.addData("4. yaw imu (control)", "%6.2f (error: %6.2f)", gyro2.getYaw(), heading - gyro2.getYaw());
        dashboardTelemetry.addData("5. yaw imu (expansion)", "%6.2f (error: %6.2f)", gyro3.getYaw(), heading - gyro3.getYaw());

        dashboardTelemetry.update();
    }

    private void setDisplaySpeed () {
        speedMsg.setValue("%4.2f", speed);
    }

    private void initNavx ()
    {
        gyro = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData);

        while ( gyro.isCalibrating() ) {
            if (! opModeIsActive()) break;
        }
        gyro.zeroYaw();
    }

    private double getOrientation () {
        return gyro1.getYaw();
    }

    /**
     * Calculate to turn speed. The turn speed to ramped up based on time, and ramped down based on
     * degrees remaining to turn.
     *
     * @param startTime time in millisecond of the start of the turn
     * @param speed speed of the turn (0-1)
     * @param degreesToGo degrees remaining to turn
     * @return motor power (0-1)
     */
    private double getRampedPower (double startTime, double speed, double degreesToGo) {

        double speedRange = Math.max(Math.abs(speed) - TURN_MIN_SPEED, 0);
        double ramUp = (startTime / TURN_RAMP_UP_TIME) * speedRange + TURN_MIN_SPEED;
        double ramDown = (Math.pow(degreesToGo, 2) / Math.pow(TURN_RAMP_DOWN_DEGREES, 2)) * speedRange + TURN_MIN_SPEED;
        return Math.min(Math.min(ramUp, ramDown), speed);
    }

    public void turnTo(double toDegrees) {

        double target;
        if (toDegrees <= 180)
            target = toDegrees;
        else
            target = -(360 - toDegrees);

        double start = getOrientation();
        double degrees = target - start;

        // Determine the shortest direction to turn
        Drive.DIRECTION direction;
        if (degrees > 0) {
            if (degrees <= 180)
                direction = Drive.DIRECTION.TURN_RIGHT;
            else
                direction = Drive.DIRECTION.TURN_LEFT;
        } else if (degrees < 0) {
            if (degrees <= -180)
                direction = Drive.DIRECTION.TURN_RIGHT;
            else
                direction = Drive.DIRECTION.TURN_LEFT;
        } else {
            return;
        }

        double lastPos = start;
        double current;
        double lastTime = 0;
        double velocity = 0;
        double degreesTurned = 0;
        double degreesEstimated;
        double degreesToTurn = Math.abs(degrees);
        degreesToTurn = Math.min(degreesToTurn, 360 - degreesToTurn);

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (opModeIsActive()) {
            double rampSpeed = getRampedPower(elapsedTime.milliseconds(), speed, degreesToTurn - degreesTurned);
            drive.moveRobot(direction, rampSpeed);
            current = getOrientation();
            if (current != lastPos) {
                double currentTime = elapsedTime.milliseconds();
                double diff = Math.abs(current - lastPos);
                diff = Math.min(diff, 360-diff);
                velocity = diff / (currentTime - lastTime);
                lastTime = currentTime;
                lastPos = current;
                degreesTurned += diff;
                degreesEstimated = 0;
            } else {
                degreesEstimated = velocity * (elapsedTime.milliseconds() - lastTime);
            }

            Logger.message(
                    //String.format("%s", direction) +
                    String.format("  to: %-6.1f", target) +
                    String.format("  from: %-6.1f", current) +
                    String.format("  to turn: %-6.2f", degreesToTurn) +
                    String.format("  turned: %-6.2f", degreesTurned) +
                    String.format("  estimated: %-6.4f", degreesEstimated) +
                    String.format("  velocity: %-6.4f", velocity) +
                    String.format("  curr: %-6.4f", current) +
                    String.format("  last: %-6.4f", lastPos));

            if (degreesTurned + degreesEstimated >= degreesToTurn)
                break;

            if (elapsedTime.seconds() > 5)
                break;
        }

        drive.stopRobot();
    }
}
