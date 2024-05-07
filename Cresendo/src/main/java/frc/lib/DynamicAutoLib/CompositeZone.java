
package frc.lib.DynamicAutoLib;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.util.GeometryUtil;

/**
 * A class for representing a zone for poses.
 * Uses Lists of Ellipse2ds and Rectangle2ds.
 */
public class CompositeZone {
    private ArrayList<Ellipse2d> ellipses = new ArrayList<>();
    private ArrayList<Rectangle2d> rectangles = new ArrayList<>();

    private ArrayList<Ellipse2d> RedEllipses = new ArrayList<>();
    private ArrayList<Rectangle2d> RedRectangles = new ArrayList<>();

    private DriverStation.Alliance currentAlliance = DriverStation.Alliance.Blue;

    public DriverStation.Alliance getCurrentAlliance() {
        return currentAlliance;
    }

    public void setAlliance(DriverStation.Alliance currentAlliance) {
        this.currentAlliance = currentAlliance;
    }

    public CompositeZone(ArrayList<Ellipse2d> ellipses, ArrayList<Rectangle2d> rectangles) {
        this.ellipses = ellipses;
        this.rectangles = rectangles;

        generateFlipped();
    
    
    }

    private void generateFlipped() {
        for (int i = 0; i < ellipses.size()-1; i++) {
            RedEllipses.set(i, new Ellipse2d(GeometryUtil.flipFieldPose(ellipses.get(i).getCenter()),
                    ellipses.get(i).getXSemiAxis(), ellipses.get(i).getYSemiAxis()));
        }

        for (int i = 0; i < rectangles.size()-1; i++) {
            RedRectangles.set(i, new Rectangle2d(GeometryUtil.flipFieldPose(rectangles.get(i).getCenter()),
                    rectangles.get(i).getXWidth(), rectangles.get(i).getYWidth()));
        }
    }

    /**
     * @param pose To pose to check
     * @return wether or not the pose is in any of the zones
     */
    public boolean isInZone(Pose2d pose) {
        if (currentAlliance == DriverStation.Alliance.Blue) {
            return isInBlueZone(pose.getTranslation());
        } else {
            return isInRedZone(pose.getTranslation());
        }
    }

    private boolean isInBlueZone(Translation2d pose) {
        for (Ellipse2d ellipse2d : ellipses) {
            if (ellipse2d.contains(pose)) {
                return true;
            }
        }
        for (Rectangle2d rectangle2d : rectangles) {
            if (rectangle2d.contains(pose)) {
                return true;
            }
        }
        return false;
    }

    private boolean isInRedZone(Translation2d pose) {
        for (Ellipse2d ellipse2d : RedEllipses) {
            if (ellipse2d.contains(pose)) {
                return true;
            }
        }
        for (Rectangle2d rectangle2d : RedRectangles) {
            if (rectangle2d.contains(pose)) {
                return true;
            }
        }
        return false;
    }

    /**
     * @param pose To pose to check
     * @return wether or not the pose is in any of the zones
     */
    public boolean isInZone(Translation2d pose) {
        if (currentAlliance == DriverStation.Alliance.Blue) {
            return isInBlueZone(pose);
        } else {
            return isInRedZone(pose);
        }
    }

    /**
     * @param pose To pose to check
     * @return wether or not the pose is in any of the zones
     */
    public boolean isInZone(Transform2d pose) {
        if (currentAlliance == DriverStation.Alliance.Blue) {
            return isInBlueZone(pose.getTranslation());
        } else {
            return isInRedZone(pose.getTranslation());
        }
    }

}
