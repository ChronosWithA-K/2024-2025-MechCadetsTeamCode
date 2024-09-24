package test.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Hello World")

public class HelloWorld extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Hello world!");
        telemetry.update();
        waitForStart();
    }
}
