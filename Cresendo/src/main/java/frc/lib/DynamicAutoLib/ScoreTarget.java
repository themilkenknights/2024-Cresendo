package frc.lib.DynamicAutoLib;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class ScoreTarget extends AutoTarget{
    public static enum TargetType{
        SHOOTER,PLACE
    }
    public ScoreTarget(Translation2d position, CompositeZone validZone, TargetType targetType) {
        super(position);
        this.targetType = targetType;
    }

    TargetType targetType;

    public static class CrescendoScoreTargets{
        ScoreTarget amp;
        ScoreTarget speaker;

        /**
         * @param ampZone valid zone for scoring in the amp
         * @param speakerZone valid zone for scoring in the speaker
         * Generates scoreTargets for blue
         */
        public CrescendoScoreTargets(CompositeZone ampZone,CompositeZone speakerZone) {
            amp = new ScoreTarget(new Translation2d(2,7), ampZone, TargetType.PLACE);
            speaker = new ScoreTarget(new Translation2d(2,7), speakerZone, TargetType.SHOOTER);//TODO: find coords
        }
        

    }
    
}
