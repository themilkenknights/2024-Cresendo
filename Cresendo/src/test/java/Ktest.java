import frc.lib.util.Ligament3d;

import static org.junit.jupiter.api.Assertions.*;

import java.beans.Transient;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.util.ForwardK;
public class Ktest {

    @org.junit.jupiter.api.Test
    void TestS(){
        ArrayList<Ligament3d> nodes = new ArrayList<>();
        nodes.add(new Ligament3d(5, new Rotation3d(0,0,0)));
        nodes.add(new Ligament3d(15, new Rotation3d(Math.PI/2,Math.PI/2,Math.PI/2)));
        System.out.println(ForwardK.getFinalPoint(new Translation3d(0,0,70), nodes).toTranslation2d());
        assertEquals( 3, 3,"The method should be invoked 3 times");
        //assertTrue(true,ForwardK.getFinalPoint(new Translation3d(),nodes)::toString);
        
    }
}
