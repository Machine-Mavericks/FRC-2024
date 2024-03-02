

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.AprilTagMap;

public class AprilTagMapTest {
    @Test
    void testCalculateRobotFieldPose() {

        // create a double array just like we would get from network tables
        double[] detection = {15.0, //AprilTag ID
                                0.0, //timestamp
                                0.0, //x in meters
                                0.0, //y in meters
                                0.0, // z in meters
                                0.0, //yaw in radians
                                0.0, //pitch in radians
                                0.0, //roll in radians
                                0.0, //range in meters
                                0.0  //bearing in radians
        };

        Pose2d calculatedPose = AprilTagMap.CalculateRobotFieldPose(detection, 4);
        
        // compare with an expected pose2d or pull out the components and compare each one to expected

    }

    @Test
    void testIntoM(){

        assertEquals(1.0, AprilTagMap.intom(39.37));
    }
}
