package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public class ArmPathPlanner {
    private double firstSegmentLength;
    private double secondSegmentLength;
    
    private final double INCREMENT = 5;

    public ArmPathPlanner(double firstSegmentLength, double secondSegmentLength) {
        this.firstSegmentLength = firstSegmentLength;
        this.secondSegmentLength = secondSegmentLength;
    }

    public Point getNextTarget(Point current, Point target) {
        Point option1 = new Point(current.getX() - INCREMENT, current.getY());
        Point option2 = new Point(current.getX(), current.getY() + INCREMENT);
        Point option3 = new Point(current.getX() + INCREMENT, current.getY());
        Point option4 = new Point(current.getX(), current.getY() - INCREMENT);
        double maxScore = getScore(current, option1, target);
        Point maxPoint = option1;

        if (current.equals(target)) {
            return current;
        }

        if (getScore(current, option2, target) > maxScore) {
            maxScore = getScore(current, option2, target);
            maxPoint = option2;
        }
        if (getScore(current, option3, target) > maxScore) {
            maxScore = getScore(current, option3, target);
            maxPoint = option3;
        }
        if (getScore(current, option4, target) > maxScore) {
            maxScore = getScore(current, option4, target);
            maxPoint = option4;
        }
        return maxPoint;
    }

    public double getScore(Point current, Point next, Point target) {
        if (!checkIfPointAllowed(next)) {
            return 0;
        }
        Vector<N2> motionVector = VecBuilder.fill(next.getX() - current.getX(), next.getY() - current.getY());
        Vector<N2> targVector = VecBuilder.fill(target.getX() - current.getX(), target.getY() - current.getY());
        
        return motionVector.dot(targVector) / Math.sqrt(motionVector.dot(motionVector));
    }

    private boolean checkIfPointAllowed(Point p) {
        if (Math.pow(p.getX() - 10, 2) + Math.pow(p.getY() - 15, 2) <= 25) {
            return false;
        }

        if (Math.pow(p.getX() - 20, 2) + Math.pow(p.getY() - 20, 2) <= 16) {
            return false;
        }
        
        if (Math.pow(p.getX() - 20, 2) + Math.pow(p.getY() - 5, 2) <= 9) {
            return false;
        }
        
        return true;
    }

    public Vector<N2> getArmAngles(Point p) {
        double elbowAngle = Math.acos(((Math.pow(p.getX(), 2) + Math.pow(p.getY(), 2)) - (Math.pow(firstSegmentLength, 2) + Math.pow(secondSegmentLength, 2)) ) / (2 * firstSegmentLength * secondSegmentLength));
        double shouderAngle = Math.atan((-secondSegmentLength * Math.sin(elbowAngle) * p.getX() + (firstSegmentLength + secondSegmentLength * Math.cos(elbowAngle)) * p.getY()) / (secondSegmentLength * Math.sin(elbowAngle) * p.getY() + (firstSegmentLength + secondSegmentLength * Math.cos(elbowAngle)) * p.getX()));
        
        Vector<N2> armAngles = VecBuilder.fill(Math.toDegrees(shouderAngle), Math.toDegrees(elbowAngle));

        if (p.getY() == 0.0) {
            armAngles = VecBuilder.fill(armAngles.get(0, 0) + 2 * (90 - armAngles.get(0, 0)), Math.toDegrees(-armAngles.get(1, 0)));
        }

        return armAngles;
    }
}
