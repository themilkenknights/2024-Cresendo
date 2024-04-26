# The MilkenKnights code for the FIRST Crescendo season
Here you will find all of the code for our robot Casio.
We use the Swerve Library from Phoenix 6 and Pathplanner. We use a custom auto selector, found in [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java). We also have untested limelight code in [AprilTagCommand.java](src/main/java/frc/robot/commands/AprilTagCommand.java)


### Style guide
For Java, please use the RedHat formatter. A stander variable naming scheme hasn't been decided.

### Upcoming changes to "lib/util"
We are working on an extension of the `AddressableLEDBuffer` class. It will have tools to make setting the whole strip solid and tools for animation.