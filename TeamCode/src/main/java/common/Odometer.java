package common;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Odometer  {

    private final double ODOMETER_COUNTS_PER_INCH = 2000/((48/25.4)*Math.PI); // counts per rev / wheel diameter in inches * pi

    public enum OdometerType {POD, MOTOR, UNDEFINED}
    OdometerType odometerType;
    DcMotorEx odometer;

    public Odometer (HardwareMap hardwareMap, String deviceName, OdometerType type) {

        try {
            HardwareDevice device = hardwareMap.get(deviceName);
            odometerType = type;
            odometer = (DcMotorEx) device;
            odometer.setDirection(DcMotorSimple.Direction.REVERSE);
            Logger.message("Created odometer %s (%s)", device.getDeviceName(), device.getClass().getSimpleName());

        } catch (Exception e) {
            Logger.warning("No odometer found");
            odometerType = OdometerType.UNDEFINED;
        }
    }

    public double getVelocity() {
        if (odometerType != OdometerType.UNDEFINED)
            return odometer.getVelocity();
        return 0;
    }

    public double countPerInch() {
        if (odometerType == OdometerType.POD)
            return ODOMETER_COUNTS_PER_INCH;
        return 0;
    }

    public void reset () {
        if (odometerType != OdometerType.UNDEFINED) {
            DcMotor.RunMode mode = odometer.getMode();
            odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odometer.setMode(mode);
        }
    }

    public int getCurrentPosition() {
        if (odometerType != OdometerType.UNDEFINED)
            return odometer.getCurrentPosition();
        return 0;
    }
}



