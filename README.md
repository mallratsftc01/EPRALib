## NOTICE

This repository contains the public FTC SDK for the INTO THE DEEP (2024-2025) competition season.

## Welcome!
This is the core library for FTC Team 18387, The Mallrats. We have worked hard to 
write this library, so please credit our team in your engineering notebook if you 
use our library in a competition robot.

## Requirements
This library will only work if you are already using the [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController). If you do not 
have that SDK installed, follow the steps included in the GitHub page linked above.

Additionally, this library has not been tested for compatibility with FTC Blocks.

## Installation
This library is hosted through GitHub packages. There are several steps to 
installation.
1) You must have a GitHub account to install this library as credentials are required to install GitHub packages. 
   1) Go to Settings → Developer Settings → Personal access tokens → Tokens (Classic) in GitHub.
   2) Generate a new token (classic) and make sure to enable read:packages.
   3) Save the generated token somewhere secure.
2) Add the following to the `gradle.properties` file in your FTC project. This will store the necessary credentials to download the package. If you are commiting to GitHub, make sure to exclude this file so that you do not leak the token.
```groovy
gpr.user=YOUR_GITHUB_USERNAME
gpr.key=YOUR_PERSONAL_ACCESS_TOKEN
```
3) In the `build.dependecies.gradle` file in your FTC project, add the following to the `repositories` section.
```groovy
maven {
        name = 'epralib'
        url = uri("https://maven.pkg.github.com/mallratsftc01/EPRALib")
        credentials {
            username = project.findProperty("gpr.user") ?: ("")
            password = project.findProperty("gpr.key") ?: ("")
        }
    }
```
4) In the same file, add the following to the `dependencies` section.
```groovy
implementation ('com.epra.epralib:ftclib:VERSION_NUMBER') {
        exclude group: 'org.firstinspires.ftc'
    }
```
5) Sync your FTC project with the modifications to the gradle files and the library should download.

## Versions & Documentation
Our version numbers use a simple x.y.z system. The first number, x, is only changed
for major overhauls or releases. The second number, y, signifies the stable update
the library is currently on. All versions with the same x and y will likely have
largely the same functionality. The third number, z, is for minor updates and patches.
Versions with a version number other than x.y.0 should be assumed to be untested and 
potentially unstable, unless otherwise stated in the commit notes.

We try our best to add javadocs to every class and method, but we have definitely 
missed a few. Please be patient with us and our code and report any bugs you may find.

Examples of programs for both TeleOp and Auto can be found in `org.firstinspires.ftc.examples`. These
examples can either help you learn EPRALib, or be used as templates to build your competition code off of.
We tried to make them as adaptable as possible. More examples may come in the future as we expand
the library's capabilities.

## Using EPRALib
For importing any classes from EPRALib, use the following.
```java
import com.epra.epralib.ftclib.GROUP_NAME.CLASS_NAME;
```
EPRALib has six groups, each with its own classes and functionality, some with subclasses.
This grouping is for ease of use as related classes or classes that interact are grouped together.

### Camera
The camera group of EPRALib is unfortunately underdeveloped. We did not use the camera
in the 2024–2025 season, so most of what this group contains is remnants from previous seasons.
We hope to further develop this group in the future, but it may remain underdeveloped 
if we do not determine it necessary in future seasons.

### Control
The control group of EPRALib contains classes that deal with robot control systems;
both external control via gamepads and internal control for auto or automated sequences.

the `Controller` class uses several different variants of button objects that implement
the `ButtonBase` interface to create a far more versatile rendition of the `Gamepad`
class that comes with the FTC SDK.

The `JSONReader` class reads JSONs that contain information on automated sequences.
We used this control in the 2024–2025 season, but we are currently trying to rework
it for the upcoming 2025–2026 season. The next major release will likely contain this
rework, along with other related features.

### Location
The location group deals with tracking the robot's current position on the field. 

Objects of the `Pose` class will store positional and rotational information. These
are used by many classes, both for tracking the robot's current location and plotting
out future locations such as in auto.

The `IMUExpanded` class can be instantiated with one or two IMUs. If two IMUs are 
provided (such as one from the control hub, one from the expansion hub) it will 
combine data from both to give a more accurate reading of the robot's bearing.

The `Odometry` class is designed to use several odometry pods to estimate the 
position and angle of the robot over time. It must be finely tuned and admittedly
gives imperfect results. You **must** have odometry pods on your robot for this class
to function; the input motors **cannot** be the same wheels that are driving your robot.
(We use [odometry pods from GoBilda](https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/?srsltid=AfmBOopwrWH-dUgehXGA4tMLjG1rMUBG9X-ZUxcD-wNPosKLqxUTrX8I))

### Math.Geometry
The Math.Geometry group contains a number of geometrical classes that prove essential
to everything from odometry to drive controls.

Some of the most basic geometrical concepts are represented by classes including `Point`,
`Angle`, and `Vector`. These are likely the most commonly used objects throughout the entire
library. Each has several different initiators for a variety of uses.

`Triangle`, `Circle`, `Quadrilateral`, and `PolyGroup` all implement the Shape2D 
interface. They represent 2D solids. While they are not used as much as other classes
in the Math.Geometry group, they all can detect if a point is
within their bounds, which can be invaluable for pathing and other applications.

The `Geometry` class is designed to be a counterpart to the java.lang `Math` class,
but for the geometrical objects found in this group. It holds static functions for
just about every transformation, and is constantly being expanded and updated.

### Math.Statistics
The other Math subgroup, while less developed than Math.Geometry, still holds useful
tools for dealing with array statistics on the fly.

The `RollingAverage` class deals with smoothing out data over time. It has several
different weighting modes that it uses to average out data over a certain period.
This can stop random spikes in value from causing your robot to malfunction, but
can also introduce latency, so use with caution.

The `Statistics` class is a simple counterpart to `java.lang.Math` and `Geometry`.
It has several functions for statistically analyzing arrays, but like the subgroup,
it is largely underdeveloped. Improving this section is currently low-priority for
us as it has only limited use cases.

### Movement

The movement group contains classes key to the actual functioning of a robot: 
the classes responsible for moving motors.

We have built the `Motor` interface as we hope to one-day support more motor types
beyond DcMotors. For now, it mainly serves as a support for the `MotorController` 
class. This class preserves the functions of a normal motor, but also adds in
other helpful features, such as a `PIDController`.

The `PIDController` class support several different types of PID loops to fine-tune
motor positions, angles, and even movement of the entire robot. We hope to eventually
connect it to a `PIDTuner` that would automatically adjust the gains for optimal
performance, but for now they do have to be manually tuned.

The `DriveTrain` class is responsible for everything regarding a robot's movement 
across the field. It has several different drive types adapted to different styles
of drive train and play. It also supports PID loops, controlled by a `PIDController`,
that can precisely position the robot based on odometry data.

### Storage
The storage group holds several records that store data in a way that can be easily
committed to JSON files. This enables logging for several different classes, including 
`MotorController`, `IMUExpanded`, `Controller`, and `Odometry`. These log files allow us 
to fine-tune troubleshooting. We hope to someday create an external AI agent
that can read these files to tune certain processes such as PID loops.

## Credits
This README was written by Striker-909, the lead programmer of FTC 18387. 

This README was last updated 6/30/2025, for version 1.1.12