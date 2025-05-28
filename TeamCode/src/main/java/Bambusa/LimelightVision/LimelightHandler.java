package Bambusa.LimelightVision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
* Localizes objects with an object detection model (TFModel) and scale, using Limelight3A.
* This script will be updated in the future. Go to SensorLimelight3A to learn to how use the Limelight.
*/

@TeleOp(name = "OBJECT_POSITION_DETECTION_TEST")
public class LimelightHandler extends LinearOpMode {

    private Limelight3A limelight;
    private ObjectLocalizer objectLocalizer;
    private Map<String, ObjectProperties> knownObjects;

    // --- CAMERA AND OBJECT CONSTANTS (MEASURE THESE ACCURATELY!) ---
    // TODO: Update these values for your robot and objects
    private static final double CAMERA_HEIGHT_ABOVE_GROUND_METERS = 0.3; // Example: 30cm
    private static final double CAMERA_PITCH_ANGLE_DEGREES = -15.0;   // Example: 15 degrees tilted DOWN

    // Define properties for your custom objects
    private static final ObjectProperties CUSTOM_OBJECT_TYPE_1 = new ObjectProperties(
            "MyObjectA", // Class name from your model
            0.15,        // Real width in meters (e.g., 15cm)
            0.10,        // Real height in meters (e.g., 10cm)
            0.05,        // Real depth in meters (e.g., 5cm)
            0.05         // Height of object's center from ground (e.g., height/2 if on ground)
    );
    private static final ObjectProperties CUSTOM_OBJECT_TYPE_2 = new ObjectProperties(
            "MyObjectB",
            0.20,
            0.20,
            0.20,
            0.10
    );
    // Add more object types as needed

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "lemonlight"); // Ensure "lemonlight" matches your config

        // Initialize the object localizer with your camera's parameters
        objectLocalizer = new ObjectLocalizer(CAMERA_HEIGHT_ABOVE_GROUND_METERS, CAMERA_PITCH_ANGLE_DEGREES);

        // Store known object properties in a map for easy lookup by class name
        knownObjects = new HashMap<>();
        knownObjects.put(CUSTOM_OBJECT_TYPE_1.className, CUSTOM_OBJECT_TYPE_1);
        knownObjects.put(CUSTOM_OBJECT_TYPE_2.className, CUSTOM_OBJECT_TYPE_2);
        // Add more objects to the map

        telemetry.setMsTransmissionInterval(50); // Adjust as needed

        limelight.pipelineSwitch(0); // Switch to your custom model pipeline if not default

        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.addData("Camera Height", "%.3f m", CAMERA_HEIGHT_ABOVE_GROUND_METERS);
        telemetry.addData("Camera Pitch", "%.1f deg", CAMERA_PITCH_ANGLE_DEGREES);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            // Optional: Display Limelight status
            // telemetry.addData("LL Name", "%s", status.getName());
            // telemetry.addData("LL Temp", "%.1fC", status.getTemp());
            // telemetry.addData("LL FPS", "%d", (int)status.getFps());
            // telemetry.addData("Pipeline", "Idx: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("LL Latency", "Cap:%.1f, Targ:%.1f, Parse:%.1f ms",
                        result.getCaptureLatency(), result.getTargetingLatency(), result.getParseLatency());

                // Access Detector Results (from your custom model)
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                if (detectorResults.isEmpty()) {
                    telemetry.addLine("No custom objects detected.");
                }

                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detected", "Class: %s, Area: %.2f, XDeg: %.1f, YDeg: %.1f",
                            dr.getClassName(), dr.getTargetArea(), dr.getTargetXDegrees(), dr.getTargetYDegrees());

                    // Get the properties for this specific detected object class
                    ObjectProperties currentObjectProps = knownObjects.get(dr.getClassName());

                    if (currentObjectProps != null) {
                        Point3D objectPosition = objectLocalizer.calculate3DPosition(dr, currentObjectProps);
                        if (objectPosition != null) {
                            telemetry.addData(dr.getClassName() + " Position", objectPosition.toString());
                            // Now you can use objectPosition.x, objectPosition.y, objectPosition.z
                            // for your robot's logic (e.g., navigation, aiming).
                        } else {
                            telemetry.addData(dr.getClassName() + " Position", "Calculation unreliable or indeterminate.");
                        }
                    } else {
                        telemetry.addData("Warning", "No properties defined for class: " + dr.getClassName());
                    }
                }
            } else {
                telemetry.addLine("Limelight: No data or invalid data.");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}