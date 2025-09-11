package Bambusa.LimelightVision;

/*
    In order to localize an object, we need its dimensions. This class stores these dimensions for use in
    the ObjectLocalizer class. Example of object localization is given in the LimelightHandler class.
 */

public class  ObjectProperties {
    public String className;
    public double realWidthMeters;   // Real-world width of the object in meters
    public double realHeightMeters;  // Real-world height of the object in meters
    public double realDepthMeters;   // Real-world depth of the object in meters
    public double centerHeightAboveGroundMeters; // Height of the object's center from the ground in meters

    public ObjectProperties(String className, double realWidthMeters, double realHeightMeters, double realDepthMeters, double centerHeightAboveGroundMeters) {
        this.className = className;
        this.realWidthMeters = realWidthMeters;
        this.realHeightMeters = realHeightMeters;
        this.realDepthMeters = realDepthMeters;
        this.centerHeightAboveGroundMeters = centerHeightAboveGroundMeters;
    }
}