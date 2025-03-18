package Bambusa;

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

// TODO: Test April Tags and Use Other Vision Methods

/*
* This class is meant to provide an example for handling vision.
* As of now, all it can handle are april tags, but I hope to improve that in the future, such as OpenCV.
* Thanks to FTC 7588 for providing this awesome tutorial about april tags: https://www.youtube.com/watch?v=CjoXoygzXzI
*
* Check out LimeLight.
* Goated tutorial at: https://www.youtube.com/watch?v=IHRZDPQ-_l8
* Buy at: https://www.andymark.com/products/limelight-3a#:~:text=Product%20Overview-,Limelight%203A%20is%20a%20plug%20and%20play%2C%20zero%2Dcode%20perception,teams%20of%20all%20experience%20levels.
*
* Helpful Videos On OpenCV:
* https://www.youtube.com/watch?v=JO7dqzJi8lw
* https://www.youtube.com/watch?v=547ZUZiYfQE&t=264s
* https://www.youtube.com/watch?v=Ojzb_fQ4a-U
* https://www.youtube.com/watch?v=mhUTWIPsxFE
*/

@TeleOp
@Config
public class Vision extends LinearOpMode {
    // Telemetry For FTC Dashboard
    TelemetryPacket packet;
    FtcDashboard dashboard;

    // April Tag Processing
    VisionPortal vision;
    AprilTagProcessor processor;

    @Override
    public void runOpMode() throws InterruptedException {

        // April Tag Settings
        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // Camera Settings
        vision = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(500, 500))
                .build();

        // Starting And Stopping
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // April Tag Handling
            if (!processor.getDetections().isEmpty()) {
                AprilTagDetection tag = processor.getDetections().get(0);

                // Now, you can access the april tag position relative to the camera!
                // Examples: tag.ftcPose.x or tag.ftcPose.yaw;

                // April Tag Debugging
                packet.put("x: ", tag.ftcPose.x);
                packet.put("y: ", tag.ftcPose.y);
                packet.put("z: ", tag.ftcPose.z);
                packet.put("roll: ", tag.ftcPose.roll);
                packet.put("pitch: ", tag.ftcPose.pitch);
                packet.put("yaw: ", tag.ftcPose.yaw);
            }

            // Telemetry (Dashboard)
            packet.put("Dashboard Telemetry Working: ", true);
            dashboard.sendTelemetryPacket(packet);

            // Telemetry (Driver Station)
            telemetry.addData("Sample Telemetry Working: ", true);
            telemetry.update();
        }
    }
}