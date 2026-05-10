package org.firstinspires.ftc.moon.planner;

/**
 * FTC Field constants and coordinate system utilities.
 * FTC Coordinate System (from PedroPath):
 * - Origin at center of field (72, 72) in inches
 * - X increases to the right (Red alliance perspective)
 * - Y increases forward (toward opponent alliance wall)
 * - Heading: 0 = pointing forward (+Y), PI = pointing backward (-Y)
 * 
 * Internal HybridA* Coordinate System:
 * - Origin at bottom-left corner of the field
 * - X increases right, Y increases up
 * - Heading: 0 = pointing right (+X), counter-clockwise positive
 */
@SuppressWarnings("ALL")
public class FTCConstants {
    
    // Field dimensions (FTC standard: 12ft x 12ft = 144in x 144in)
    public static final double FIELD_WIDTH_INCHES = 144.0;
    public static final double FIELD_HEIGHT_INCHES = 144.0;
    public static final double FIELD_CENTER = FIELD_WIDTH_INCHES / 2.0;  // 72 inches
    
    // Default grid resolutions (in inches)
    public static final double DEFAULT_MAP_GRID_RESOLUTION = 0.5;   // 0.5 inch per grid cell
    public static final double DEFAULT_STATE_GRID_RESOLUTION = 1.0; // 1.0 inch per state cell
    
    // Vehicle shape defaults (in inches)
    public static final double DEFAULT_LENGTH = 18.0;     // Robot length
    public static final double DEFAULT_WIDTH = 18.0;     // Robot width
    public static final double DEFAULT_REAR_AXLE_TO_REAR = 6.0;  // Rear axle to rear bumper
    
    // Dead wheel odometrypod offsets (for localization)
    public static final double FORWARD_POD_Y = 0.945;    // inches behind center
    public static final double STRAFE_POD_X = 1.488;     // inches right of center
    
    /**
     * Convert HybridA* internal coordinates to FTC coordinates.
     * 
     * @param internalX X in HybridA* (0 = left edge of field, increasing right)
     * @param internalY Y in HybridA* (0 = bottom edge of field, increasing up)
     * @param headingRad Heading in HybridA* (0 = pointing right, CCW positive)
     * @return Vec3d with FTC coordinates (x, y, heading)
     */
    public static Vec3d internalToFTC(double internalX, double internalY, double headingRad) {
        // HybridA* uses: origin at bottom-left, heading 0=right, CCW positive
        // FTC/Pedro uses: origin at center, heading 0=forward (+Y), CW positive
        
        // Convert position: rotate 90° CW and shift to center
        double ftcX = internalY + FIELD_CENTER;
        double ftcY = -internalX + FIELD_CENTER;  // Note: -internalX for mirror
        
        // Convert heading: 0=right -> 0=forward, then invert (rotate 90° CW)
        // heading 0 in internal = +X direction = +Y direction in FTC = forward = 0 in Pedro
        // but in Pedro, heading 0 = forward, and we need to invert direction
        double ftcHeading = -headingRad + Math.PI / 2;
        
        // Normalize to [-PI, PI]
        ftcHeading = normalizeAngle(ftcHeading);
        
        return new Vec3d(ftcX, ftcY, ftcHeading);
    }
    
    /**
     * Convert FTC coordinates to HybridA* internal coordinates.
     * 
     * @param ftcX FTC X (0 at left edge, 144 at right edge)
     * @param ftcY FTC Y (0 at back edge, 144 at front edge)  
     * @param ftcHeading FTC heading (0 = forward, PI = backward)
     * @return Vec3d with internal coordinates
     */
    public static Vec3d ftcToInternal(double ftcX, double ftcY, double ftcHeading) {
        // Inverse of internalToFTC
        // Position
        double internalX = -(ftcY - FIELD_CENTER);
        double internalY = ftcX - FIELD_CENTER;
        
        // Heading: invert the transform
        // ftcHeading = -internalHeading + PI/2
        // -internalHeading = ftcHeading - PI/2
        // internalHeading = PI/2 - ftcHeading
        double internalHeading = Math.PI / 2 - ftcHeading;
        
        // Normalize
        internalHeading = normalizeAngle(internalHeading);
        
        return new Vec3d(internalX, internalY, internalHeading);
    }
    
    /**
     * Convert FTC coordinates to PedroPath Pose.
     */
    public static com.pedropathing.geometry.Pose ftcToPedro(double ftcX, double ftcY, double ftcHeading) {
        // FTC to Pedro: rotate -90° and add offset
        double pedroX = ftcY - FIELD_CENTER;
        double pedroY = ftcX - FIELD_CENTER;
        double pedroHeading = ftcHeading - Math.PI / 2;
        
        return new com.pedropathing.geometry.Pose(pedroX, pedroY, pedroHeading);
    }
    
    /**
     * Convert PedroPath Pose to FTC coordinates.
     */
    public static Vec3d pedroToFTC(com.pedropathing.geometry.Pose pedroPose) {
        double ftcX = pedroPose.getY() + FIELD_CENTER;
        double ftcY = pedroPose.getX() + FIELD_CENTER;
        double ftcHeading = pedroPose.getHeading() + Math.PI / 2;
        
        return new Vec3d(ftcX, ftcY, ftcHeading);
    }
    
    /**
     * Normalize angle to [-PI, PI].
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    /**
     * Get the boundary of the field for internal coordinates.
     */
    public static double[] getInternalBounds() {
        return new double[]{0, FIELD_WIDTH_INCHES, 0, FIELD_HEIGHT_INCHES};
    }
}