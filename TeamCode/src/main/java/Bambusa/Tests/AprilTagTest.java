package Bambusa.Tests;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.util.Size;

/*
 * This is a sample april tag processor for Bambusa 6226
 * This code is meant to be copied and pasted into your Auto or TeleOp for localization purposes.
 *
 * Here are the steps necessary to set up a camera:
 * 1 - Plug in your camera into the control hub.
 * 2 - Using your Driver Station, go to configure and press Scan Devices.
 * 3 - Something like "Webcam 1" should show. You may rename it. Change Line 49 to your new name.
 * 4 - Calibrate your camera, visit:
 *      https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/camera_calibration/camera-calibration.html
 *
 * Your done!
*/

@TeleOp
@Config
public class AprilTagTest extends LinearOpMode {
    // Telemetry For FTC Dashboard
    TelemetryPacket packet;
    FtcDashboard dashboard;

    // April Tag Processing
    VisionPortal vision;
    AprilTagProcessor processor;

    @Override
    public void runOpMode() throws InterruptedException {

        // FTC Dashboard setup
        dashboard = FtcDashboard.getInstance();

        // Webcam
        WebcamName webcam;

        try {
            webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception e) {
            telemetry.addData("Error: ", "Webcam not found. Please check your configuration, and scan devices again.");
            telemetry.update();
            return;
        }

        // April Tag Processor Settings
        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // Camera Settings
        vision = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .build();

        // Waiting For Start
        waitForStart();
        if (isStopRequested()) return;

        // Starting Camera Stream To FTC Dashboard
        dashboard.startCameraStream(vision, 30);

        while (opModeIsActive()) {
            packet = new TelemetryPacket();

            // Process April Tag Detections
            if (!processor.getDetections().isEmpty()) {
                AprilTagDetection tag = processor.getDetections().get(0);

                // Check If Position Is Available
                if (tag.ftcPose != null) {
                    packet.put("x", tag.ftcPose.x);
                    packet.put("y", tag.ftcPose.y);
                    packet.put("z", tag.ftcPose.z);
                    packet.put("roll", tag.ftcPose.roll);
                    packet.put("pitch", tag.ftcPose.pitch);
                    packet.put("yaw", tag.ftcPose.yaw);
                } else {
                    packet.put("April Tag Position: ", "Not available");
                }
            }

            // Dashboard Telemetry
            packet.put("Dashboard Telemetry Working", true);
            dashboard.sendTelemetryPacket(packet);

            // Driver Station Telemetry
            telemetry.addData("Sample Telemetry Working", true);
            telemetry.update();
        }
    }
}
