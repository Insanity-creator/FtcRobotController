package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // or TeleOp
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

@Autonomous(name = "AprilTag Webcam Example", group = "Vision")
public class AprilTagWebcamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        aprilTagWebcam.updateVision();

        // Check for the Blue Scoring cluster
        AprilTagClusterDetection blueScoringCluster = aprilTagWebcam.getClusterByName("BLUE SCORING");

        if (blueScoringCluster != null) {
            aprilTagWebcam.displayTelemetry(blueScoringCluster);
        } else {
            telemetry.addData("Cluster Status", "Blue Scoring cluster not in view");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}