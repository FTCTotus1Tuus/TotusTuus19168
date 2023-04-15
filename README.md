# TotusTuus19168
ftc team 19168's 2023-4 season code

## To login

1: turn on bot and join its wifi<br />
2: go to http://192.168.43.1:8080/ <br />
3: go to the onBotJava tab to start coding <br />

## Inital look at the code

[DarienOpMode](ftc/teamcode/DarienOpMode.java) is main class for our auto. It contains the methods to run the auto as well as constant variables and the system to store the global coords. <Br />
[ControllerMode](ftc/teamcode/ControllerMode.java) is the main teleop program that we run. It contains the methods to run the robot with controllers. A base code to move the wheels will stay the same and we add code for the attachments added each season.<br />
[BlueCorners](ftc/teamcode/StateBlueCorners.java) and [RedCorners](ftc/teamcode/StateRedCorners.java) are the files that contains the individual commands to run the autonomous. The different files contain the different positions for the robot to start. Over the season we will freeze each auto program we use during the season using the naming convention:<br />
Meet1Blue, Meet2Blue, Meet3Blue, RegionalsBlue, StateBlue, WorldBlue, WeWonBlue

