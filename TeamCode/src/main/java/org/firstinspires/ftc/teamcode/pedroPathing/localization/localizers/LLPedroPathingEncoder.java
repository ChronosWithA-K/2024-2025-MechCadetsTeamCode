package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class LLPedroPathingEncoder extends DriveEncoderLocalizer {
    private Pose limelightPose;

    private IMU imu = null;

    private Limelight3A limelight;

    /**
     * This creates a new DriveEncoderLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public LLPedroPathingEncoder(HardwareMap map) {
        super(map);
        limelightPose = null;

        imu = map.get(IMU.class, "imu");

        limelight = map.get(Limelight3A.class, "limelight");

        // Start the limelight polling for data (getLatestResult() will return null without this)
        limelight.start();
        limelight.pipelineSwitch(9);
    }

    /**
     * This creates a new DriveEncoderLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map          the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public LLPedroPathingEncoder(HardwareMap map, Pose setStartPose) {
        super(map, setStartPose);
    }

    @Override
    public Pose getPose() {
        if (limelightPose != null) {
            return limelightPose;
        }
        return super.getPose();
    }

    /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders. Then, the robot's global change in position is calculated
     * using the pose exponential method.
     */
    @Override
    public void update() {
        super.update();

        LLResult limelightResult = limelight.getLatestResult();

        if (limelightResult != null) {
            limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());

            if (limelightResult.isValid()) {
                Pose3D botpose = limelightResult.getBotpose();
                double x = botpose.getPosition().x;
                x /= 0.0254;
                double y = botpose.getPosition().y;
                y /= 0.0254;
                limelightPose = new Pose(x,y, botpose.getOrientation().getYaw());
                //  setPose(limelightPose);
            } else {
                limelightPose = null;
            }
        } else {
            limelightPose = null;
        }
    }
}
