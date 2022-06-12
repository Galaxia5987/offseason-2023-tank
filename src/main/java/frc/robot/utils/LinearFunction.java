package frc.robot.utils;

public class LinearFunction {
    private final double slope;
    private final double offset;
    private final double min;
    private final double max;

    public LinearFunction(double slope, double offset, double min, double max) {
        this.slope = slope;
        this.offset = offset;
        this.min = min;
        this.max = max;
    }

    public boolean inRange(double x) {
        return x >= min && x <= max;
    }

    public double apply(double x) {
        if (!inRange(x)) {
            System.out.println("Input not in function range.");
            return 0;
        }
        return slope * x + offset;
    }
}
