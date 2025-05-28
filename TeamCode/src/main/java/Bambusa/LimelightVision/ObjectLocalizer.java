package Bambusa.LimelightVision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

/*
 * Localizes objects detected through the Limelight 3A. Tested in LimelightHandler.
*/

public class ObjectLocalizer {

    private final double cameraHeightAboveGroundMeters; // Height of camera lens from ground
    private final double cameraPitchAngleRadians;     // Camera pitch: positive if angled up, negative if angled down

    /**
     * Constructor for ObjectLocalizer.
     * @param cameraHeightAboveGroundMeters Height of the camera lens from the ground (meters).
     * @param cameraPitchAngleDegrees Pitch angle of the camera relative to horizontal (degrees).
     * Positive if angled up, negative if angled down.
     */
    public ObjectLocalizer(double cameraHeightAboveGroundMeters, double cameraPitchAngleDegrees) {
        this.cameraHeightAboveGroundMeters = cameraHeightAboveGroundMeters;
        this.cameraPitchAngleRadians = Math.toRadians(cameraPitchAngleDegrees);
    }

    /**
     * Calculates the 3D position of a detected object relative to the robot.
     * Robot coordinate system: +X is right, +Y is forward, +Z is up.
     *
     * @param detection The DetectorResult from Limelight for a specific object.
     * @param objectProps Known properties of the detected object type.
     * @return Point3D representing the object's center position (X,Y,Z in meters), or null if calculation is unreliable.
     */
    public Point3D calculate3DPosition(LLResultTypes.DetectorResult detection, ObjectProperties objectProps) {
        if (detection == null || objectProps == null) {
            return null;
        }

        double tx_degrees = detection.getTargetXDegrees(); // Horizontal angle from crosshair to target
        double ty_degrees = detection.getTargetYDegrees(); // Vertical angle from crosshair to target

        double tx_radians = Math.toRadians(tx_degrees);
        double ty_radians = Math.toRadians(ty_degrees);

        // h1: Camera height above ground
        // h2: Object's center height above ground
        // a1: Camera pitch angle (from horizontal, positive up)
        // a2: Target's vertical angle from camera's axis (ty_radians, positive up)
        double h1_cameraHeight = this.cameraHeightAboveGroundMeters;
        double h2_objectCenterHeight = objectProps.centerHeightAboveGroundMeters;
        double a1_cameraPitch = this.cameraPitchAngleRadians;
        double a2_limelight_ty = ty_radians;

        // Total vertical angle from the robot's horizontal plane to the target
        double totalVerticalAngleToTarget = a1_cameraPitch + a2_limelight_ty;

        // Difference in height between object's center and camera's height
        // deltaH = h2_objectCenterHeight - h1_cameraHeight
        double deltaH = h2_objectCenterHeight - h1_cameraHeight;

        // Handle edge cases for calculation stability:
        // Case 1: Object and camera are effectively at the same height.
        if (Math.abs(deltaH) < 0.01) { // Less than 1cm difference
            if (Math.abs(totalVerticalAngleToTarget) < Math.toRadians(1.0)) { // And looking nearly horizontal
                // Indeterminate: Cannot calculate distance accurately.
                System.err.println("Localization: Indeterminate - Object and camera at same height, near zero vertical angle.");
                return null;
            }
            // If deltaH is ~0 but angle is significant, implies Y is ~0 (directly to side, above/below on that plane).
            // Formula `deltaH / tan(angle)` will correctly result in ~0 for robotY_forward.
        }

        // Case 2: Looking almost straight horizontally at an object not at the same height.
        // This would result in a very large (theoretically infinite) distance.
        if (Math.abs(totalVerticalAngleToTarget) < Math.toRadians(1.0) && Math.abs(deltaH) >= 0.01) {
            System.err.println("Localization: Unreliable - Very shallow angle to object at different height, distance too large.");
            return null; // Or handle as "very far away"
        }

        // Case 3: Angle is too steep (looking almost straight up/down) making tan problematic (approach infinity).
        // tan(90) or tan(-90) is undefined.
        if (Math.abs(totalVerticalAngleToTarget) >= Math.toRadians(89.0)) {
            System.err.println("Localization: Unreliable - Angle too steep (>= 89 degrees).");
            // If deltaH is non-zero, Y distance will be very small. X might be large.
            // This can be a valid scenario if object is directly above/below.
            // For now, we'll allow it but it could be sensitive.
            // If deltaH is exactly zero, this is caught by indeterminate case if angle also zero.
        }


        // Calculate forward distance (robot's +Y axis)
        // robotY_forward = (object_center_height - camera_height) / tan(total_vertical_angle_to_target)
        double robotY_forward = deltaH / Math.tan(totalVerticalAngleToTarget);

        // Calculate sideways distance (robot's +X axis)
        // robotX_right = forward_distance * tan(horizontal_angle_to_target)
        double robotX_right = robotY_forward * Math.tan(tx_radians);

        // Vertical position of object's center (robot's +Z axis)
        // This is simply the known height of the object's center above ground.
        double robotZ_up_objectCenter = h2_objectCenterHeight;

        return new Point3D(robotX_right, robotY_forward, robotZ_up_objectCenter);
    }
}