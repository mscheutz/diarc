package edu.tufts.hrilab.packing;

/**
 * A class to represent a camera's state, including its intrinsic parameters,
 * and to calculate transformations in world coordinates.
 */
public class ComplicatedCamera {

    // intrinsic camera parameters, matrix K from calibration
    private double a;
    private double cx, cy;
    private final double[][] K = new double[3][3];

    // extrinsic alignment parameters, need to be calculated based on detection
    private double alignmentRotationZ; // rotation around the optical axis in radians
    private Vec2 alignmentShift;       // shift in the camera's local XY plane in meters

    public ComplicatedCamera() {
        this.a = 0;
        this.cx = 0;
        this.cy = 0;
        this.alignmentRotationZ = 0;
        this.alignmentShift = new Vec2(0, 0);
        updateK();
    }

    /**
     * Updates the intrinsic parameters and recalculates the camera matrix K:
     * @param a The calculated a = Z*L/W
     * @param cx The x-coordinate of the image center
     * @param cy The y-coordinate of the image center
     */
    public void updateIntrinsics(double a, double cx, double cy) {
        this.a = a;
        this.cx = cx;
        this.cy = cy;
        updateK();
    }

    /**
     * Updates the alignment transformation needed to center and align the object:
     * @param rotationAngleRad The rotation around the camera's Z-axis, i.e., optical axis
     * @param shiftX The real-world horizontal shift in the camera's local plane
     * @param shiftY The real-world vertical shift in the camera's local plane
     */
    public void updateAlignment(double rotationAngleRad, double shiftX, double shiftY) {
        this.alignmentRotationZ = rotationAngleRad;
        this.alignmentShift = new Vec2(shiftX, shiftY);
    }

    /**
     * calculates the new world pose (position and rotation) of the camera
     * by applying the stored alignment transformation
     *
     * @param currentPosition The camera's current position in world coordinates
     * @param currentRotation The camera's current rotation in world coordinates as Euler angles (in radians).
     * @return A Pose object containing the new position and rotation.
     */
    public Pose getAlignedCameraPose(Vec3 currentPosition, Vec3 currentRotation) {
        // create a rotation matrix from the camera's current world rotation (Euler angles)
        double[][] worldRotationMatrix = eulerToRotationMatrix(currentRotation);

        // the alignment shift is in the camera's local XY plane, transform it to a world-space vector
        // the local shift vector is (shift.x, shift.y, 0)
        double worldShiftX = worldRotationMatrix[0][0] * alignmentShift.x + worldRotationMatrix[0][1] * alignmentShift.y;
        double worldShiftY = worldRotationMatrix[1][0] * alignmentShift.x + worldRotationMatrix[1][1] * alignmentShift.y;
        double worldShiftZ = worldRotationMatrix[2][0] * alignmentShift.x + worldRotationMatrix[2][1] * alignmentShift.y;

        // calculate the new world position by adding the world shift
        Vec3 newPosition = new Vec3(
            currentPosition.x + worldShiftX,
            currentPosition.y + worldShiftY,
            currentPosition.z + worldShiftZ
        );

        // the alignment rotation is around the camera's local Z-axis (optical axis)
        // multiply the current world rotation by the alignment rotation
	// for Euler angles, this is equivalent to adding the alignment rotation
	// to the current Z-axis rotation
        Vec3 newRotation = new Vec3(
            currentRotation.x,
            currentRotation.y,
            currentRotation.z + this.alignmentRotationZ
        );

        return new Pose(newPosition, newRotation);
    }

    private void updateK() {
        K[0][0] = a;  K[0][1] = 0;  K[0][2] = cx;
        K[1][0] = 0;  K[1][1] = a;  K[1][2] = cy;
        K[2][0] = 0;  K[2][1] = 0;  K[2][2] = 1;
    }

    private double[][] eulerToRotationMatrix(Vec3 euler) {
        double cosX = Math.cos(euler.x);
        double sinX = Math.sin(euler.x);
        double cosY = Math.cos(euler.y);
        double sinY = Math.sin(euler.y);
        double cosZ = Math.cos(euler.z);
        double sinZ = Math.sin(euler.z);

        double[][] Rz = {{cosZ, -sinZ, 0}, {sinZ, cosZ, 0}, {0, 0, 1}};
        double[][] Ry = {{cosY, 0, sinY}, {0, 1, 0}, {-sinY, 0, cosY}};
        double[][] Rx = {{1, 0, 0}, {0, cosX, -sinX}, {0, sinX, cosX}};
        
        // Combine rotations: Z * Y * X
        return multiplyMatrices(multiplyMatrices(Rz, Ry), Rx);
    }

    private double[][] multiplyMatrices(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    @Override
    public String toString() {
        return String.format("Camera State:%n  K Matrix:%n    [%.2f, %.2f, %.2f]%n    [%.2f, %.2f, %.2f]%n    [%.2f, %.2f, %.2f]%n  Alignment:%n    Shift (X,Y): (%.4f m, %.4f m)%n    Rotation (Z): %.2f degrees",
            K[0][0], K[0][1], K[0][2],
            K[1][0], K[1][1], K[1][2],
            K[2][0], K[2][1], K[2][2],
            alignmentShift.x, alignmentShift.y, Math.toDegrees(alignmentRotationZ));
    }

}
