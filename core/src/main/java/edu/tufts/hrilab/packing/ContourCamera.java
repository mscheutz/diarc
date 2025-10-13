package edu.tufts.hrilab.packing;

public class ContourCamera {
    // Intrinsic parameters
    private double a;  // Scaling factor (focal length in pixels)
    private double cx, cy; // Principal point (image center)

    // Extrinsic parameters (alignment)
    private double rotation; // In radians
    private double shiftX, shiftY; // In meters

    public void updateIntrinsics(double a, double cx, double cy) {
        this.a = a;
        this.cx = cx;
        this.cy = cy;
    }

    public void updateAlignment(double rotation, double shiftX, double shiftY) {
        this.rotation = rotation;
        this.shiftX = shiftX;
        this.shiftY = shiftY;
    }

    @Override
    public String toString() {
        return String.format(
            "Camera Parameters:\n" +
            "  Intrinsic Matrix (K):\n" +
            "    [ %.2f,   0.00,   %.2f ]\n" +
            "    [   0.00,   %.2f,   %.2f ]\n" +
            "    [   0.00,   0.00,     1.00 ]\n" +
            "  Alignment:\n" +
            "    Rotation: %.2f degrees\n" +
            "    Shift X:  %.4f meters\n" +
            "    Shift Y:  %.4f meters",
            a, cx, a, cy, Math.toDegrees(rotation), shiftX, shiftY
        );
    }
}