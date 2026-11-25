package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

@Autonomous(name = "AprilTag Webcam Example", group = "Vision")
public class AprilTagWebcamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // Set to true for Blue Alliance, false for Red Alliance
    private boolean isBlueAlliance = true;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        aprilTagWebcam.updateVision();

        // Automatically choose nearest blue or red cluster based on your alliance variable
        AprilTagClusterDetection targetCluster = isBlueAlliance ?
                aprilTagWebcam.getNearestBlueCluster() :
                aprilTagWebcam.getNearestRedCluster();

        if (targetCluster != null) {
            aprilTagWebcam.displayTelemetry(targetCluster);
            telemetry.addData("Target Locked", targetCluster.metadata.name);
            telemetry.addData("Distance (cm)", "%6.1f", targetCluster.ftcPose.range);
        } else {
            telemetry.addData("Cluster Status", (isBlueAlliance ? "Blue" : "Red") + " cluster not in view");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}