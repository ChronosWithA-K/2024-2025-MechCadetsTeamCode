package test.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;

import common.Drive;
import common.Logger;
import utils.Increment;

@TeleOp(name="Drive To Object", group="Test")
@SuppressWarnings("unused")
public class DriveToObject extends LinearOpMode {

  private final String settingsPath = "/temp/driveToObject.txt";

  private static final double MIN_SPEED = 0.20;
  private static final double MAX_SPEED = 0.90;
  private static final double MIN_INCHES = 0;
  private static final double MAX_INCHES = 12;

  private double inches = 1.5;
  private double speed = 0.25;

  Telemetry.Item inchesMsg;
  Telemetry.Item speedMsg;
  Telemetry.Item distanceMsg;

  Drive drive;

  @Override
  public void runOpMode() {

    telemetry.addData("Status", "Press start");
    telemetry.update();
    telemetry.setAutoClear(false);

    Increment inchesIncrement = new Increment(0.1, 0.5, 1);
    Increment speedIncrement = new Increment(0.01, 0.05, 0.1);

    drive = new Drive(this);
    drive.start();

    readSettings();

    waitForStart();

    inchesMsg =  telemetry.addData("Distance from object to stop (inches)", 0);
    speedMsg =  telemetry.addData("Speed", 0);
    distanceMsg =  telemetry.addData("Distance from object", 0);

    setDisplayInches();
    setDisplaySpeed();
    setDisplayDistance();

    telemetry.addData("\nMove To Object Controls", "\n" +
            "  a - drive to object\n" +
            "  dpad up - increase the stop at distance\n" +
            "  dpad down - decrease the stop at distance \n" +
            "  dpad left - decrease speed\n" +
            "  dpad right - increase speed\n" +
            "  left trigger - manual drive acceleration\n" +
            "  left stick - manual drive manual steer\n" +
            "  right stick - manual drive rotate\n");

    telemetry.update();

    while (opModeIsActive()) {

      if (gamepad1.a) {
        // drive to the object
        boolean found = drive.moveToObject(speed, inches, 10000);
        Logger.message("object found: %b", found);
        while (gamepad1.a && opModeIsActive()) {
          sleep(10);
        }

      } else if (gamepad1.dpad_up) {
        // increase the distance from the object to stop at
        inchesIncrement.reset();
        while (gamepad1.dpad_up) {
          inches = Math.min(inches + inchesIncrement.get(), MAX_INCHES);
          setDisplayInches();
          telemetry.update();
        }

      } else if (gamepad1.dpad_down) {
        // increase the distance from the object to stop at
        inchesIncrement.reset();
        while (gamepad1.dpad_down) {
          inches = Math.max(inches - inchesIncrement.get(), MIN_INCHES);
          setDisplayInches();
          telemetry.update();
        }

      } else if (gamepad1.dpad_right) {
        // increase the speed to travel
        speedIncrement.reset();
        while (gamepad1.dpad_right) {
          speed = Math.min(speed + speedIncrement.get(), MAX_SPEED);
          setDisplaySpeed();
          telemetry.update();
        }

      } else if (gamepad1.dpad_left) {
        // decrease the speed the travel
        speedIncrement.reset();
        while (gamepad1.dpad_left) {
          speed = Math.max(speed - speedIncrement.get(), MIN_SPEED);
          setDisplaySpeed();
          telemetry.update();
        }
      }

      setDisplayDistance();
      telemetry.update();
    }

    writeSettings();
  }

  private void setDisplayInches () {
    inchesMsg.setValue("%4.2f", inches);
  }

  private void setDisplaySpeed () {
    speedMsg.setValue("%4.2f", speed);
  }

  private void setDisplayDistance () {
    double distance = drive.distanceToObject();
    distanceMsg.setValue("%6.2f", distance);
  }

  // Write the setting to a file
  private void writeSettings () {

    JSONObject settings = new JSONObject();

    try {
      settings.put("Inches", inches);
      settings.put("Speed", speed);

    } catch (JSONException jsonException) {
      Logger.message(jsonException.getMessage());
      return;
    }

    Logger.message(settings.toString());

    try {
      String path = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), settingsPath);
      Logger.message ("Writing settings data to %s", path);

      FileWriter fileWriter = new FileWriter(path);

      fileWriter.write(settings.toString());
      fileWriter.flush();
      fileWriter.close();

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  // Read the settings from a file
  private void readSettings() {

    String path = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), settingsPath);
    Logger.message ("Reading calibration data from %s", path);

    String str;
    try {
      str = readFile(path);
      Logger.message(str);
    } catch (java.io.IOException exception) {
      return;
    }

    try {
      JSONObject settings = new JSONObject(str);
      inches = settings.getDouble("Inches");
      speed = settings.getDouble("Speed");

    } catch (JSONException jsonException) {
      Logger.message(jsonException.getMessage());
    }
  }

  // Read the contents of a file into a string
  private String readFile(String path) throws java.io.IOException
  {
    File file = new File(path);
    byte[] b  = new byte[(int)file.length()];
    int total = 0;

    try {
      FileInputStream in = new FileInputStream(file);
      while (total < b.length) {
        int bytesRead = in.read(b, total, b.length - total);
        if (bytesRead == -1) {
          break;
        }
        total += bytesRead;
      }
      in.close();

    } catch (java.io.IOException exception) {
      Logger.message(exception.getMessage());
      throw exception;
    }

    return new String(b);
  }
}

