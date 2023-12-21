package fastRobot_ROS2_HUMBLE;

public class AngleConverter {

    public static double[] degreesToRadians(double[] degrees) {
        double[] radians = new double[degrees.length];

        for (int i = 0; i < degrees.length; i++) {
            radians[i] = degrees[i] * Math.PI / 180.0;
        }

        return radians;
}
    };