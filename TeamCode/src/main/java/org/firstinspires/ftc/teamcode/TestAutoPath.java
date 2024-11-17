package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Test Path")
public class TestAutoPath extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor viperSlideMotor = null; // Declare viper slide motor
    private Servo specimenClaw = null; // Declare specimen claw servo


    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viper_slide_motor"); // Initialize viper slide motor
        specimenClaw = hardwareMap.get(Servo.class, "specimen_claw_servo"); // Initialize specimen claw servo


        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double leftPower = 0.0;
            double rightPower = 0.0;


            // Move forward for 1 second
            if (runtime.milliseconds() < 1000) {
                leftPower = 1;
                rightPower = 1;
            }
            // Turn left for 1 second
            else if (runtime.milliseconds() < 2000) {
                leftPower = -1;
                rightPower = 1;
            }
            // Move forward again for 1 second
            else if (runtime.milliseconds() < 3000) {
                leftPower = 1;
                rightPower = 1;
            }
            // Turn right for 1 second
            else if (runtime.milliseconds() < 4000) {
                leftPower = 1;
                rightPower = -1;
            } else {
                // Stop after completing the movements
                leftPower = 0;
                rightPower = 0;
            }
            viperSlideMotor.setPower(0.0); // Set viper slide motor to full power (1.0)
            specimenClaw.setPosition(0.5); // Set specimen claw to middle position (0.5)


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }


        // After the movement sequence is complete, set the viper slide motor and specimen claw
        viperSlideMotor.setPower(0.0); // Set viper slide motor to full power (1.0)
        specimenClaw.setPosition(0.5); // Set specimen claw to middle position (0.5)


        // Show status on telemetry
        telemetry.addData("Viper Slide Motor", "Power: %.2f", viperSlideMotor.getPower());
        telemetry.addData("Specimen Claw", "Position: %.2f", specimenClaw.getPosition());
        telemetry.update();


        // Sleep for 2 seconds to ensure the claw and slide have time to move
        sleep(2000);
    }
}





