## Overview

The `storage.autonomous` subgroup contains the `AutoProgram` class, A very powerful class that can represent an entire autonomous program. An auto program is based on a directory of `JSON`s which represent individual movements, program structure, and conditionals that control flow.

An example of an auto directory's structure:

```
auto
├ program.json
├ conditionals.json
├ constants.json
├ init.json
├ FIRST_MOVEMENT.json
├ SECOND_MOVEMENT.json
┆
└ LAST_MOVEMENT.json
```
## Auto Steps

The most basic unit of an auto program is a single `AutoStep`. An auto step is essentially a single instruction for the robot. It is composed of a comment (which has no functional purpose), an end condition (see the section on conditionals), a `DriveTrainAutoModule` (DTAM), and any number of `MotorControllerAutoModules` (MCAM).

An example of an auto step's structure:

```json
{
       "comment": "Moves forward and shoots",
       "endCondition": "DONE_SHOOTING",
       "driveTrainModule": {
           "driveMode": "direct_drive",
           "maxPower": "[DRIVE.MAX_POWER]",
           "x": 0.5,
           "y": 0.25,
           "angle": 90,
           "posTolerance": 0,
           "angleTolerance": "[DRIVE.ANGLE_TOLERANCE]"
       },
       "motorControllerModules": [
       {
           "id": "Shooter",
           "motorMode": "direct_power",
           "target": 0,
           "tolerance": 0,
           "maxPower": "[SHOOTER.POWER]"
       }
       ]
     }
```

The DTAM controls how the `DriveTrain` moves. The DTAM has four `driveMode`s:
- `none` will halt the robot, not moving at all.

 - `direct_drive` drives the robot at `maxPower` in the direction specified by `x` and `y`,
 treated as a vector relative to the field.
 It will also rotate to the specified `angle` within the `angleTolerance`
 (1 being no precision and 0 being infinite precision), relative to the field.
 This drive mode uses the `DriveTrain`'s `fieldOrientedMecanumDrive`.

 - `precise_targeted_drive` drives the robot to the `Pose` on the field specified by `x`, `y`, and the `angle`.
 Precision is specified by `posTolerance` and `angleTolerance` (1 being no precision and 0 being infinite precision)
 for both. This drive mode uses the `DriveTrain`'s `posPIDMecanumDrive`.

 - `non_precise_targeted_drive` drives the robot **in the direction of** the target position specified by `x` and `y`. The `posTolerance` is the minimum distance between the robot and the target position that is considered "close enough". `angle` is handled the same way as the other drive modes. This drive mode uses the `DriveTrain`'s `posPIDMecanumDrive`.

Each MCAM gives instructions to a single `MotorController`. They should **not** be used for drive train motors, as that can interfere with the instructions from the DTAM. Similar to the DTAM, MCAMs have three `motorModes`: 

- `none` will not drive the motor at all, setting the power to 0.

- `direct_power` moves the motor at `maxPower` constantly. `maxPower` should be between -1 and 1.

- `targeted` moves the motor to `target` in motor-specific ticks. The absolute maximum power will be
     `maxPower`. The tolerance for reaching `target` is `tolerance` (between 0 and 1).

Both DTAMs and MCAMs can have values set by strings that reference constants or internal variables. They use the same system as Conditionals (explained later). This allows for variables such as shooter velocity or maximum drive speed to be updated dynamically.

## Movement Files

The next step up from auto steps, a movement file is a `JSON` file containing a list of auto steps. These steps are executed in order, once called after the previous's end condition was `True`.

Movement files are organized in the `program.json` file. The first movement file used will always be `init.json`. Once a movement file is complete, the next movement file is determined by a list of condition-filename pairs. These look something like `"MOVEMENT_1": "CONDITION_1"`. These are treated like a switch statement, so the first condition that is true will cause the associated movement file to be run.

An example of `program.json`'s structure:

```json
{
     "init.json": { "TRUE": "FIRST_MOVEMENT.json" },
     "FIRST_MOVEMENT.json": {
         "CONDITIONAL_NAME": "NEXT_MOVEMENT_IF_CONDITION.json",
         ...
     },
     "SECOND_MOVEMENT.json": [ ... ],
     ...
     "LAST_MOVEMENT.json": []
 }
```

## Conditionals

A key feature of this implementation of an auto program is the adaptability as a result of conditionals. These blocks of pseudocode, defined in `conditionals.json`, take data from the robot to determine how the auto should proceed.

An example of `conditionals.json`'s structure:

```json
{
     "CONDITIONAL_1_NAME": "[OBJECT.DATA] == 0",
     "CONDITIONAL_2_NAME": "[OBJECT_1.DATA] >= [OBJECT_2.DATA]",
     "CONDITIONAL_3_NAME": "[OBJECT_1.DATA_1] < [OBJECT_2.DATA_1] && [OBJECT_1.DATA_2] != [OBJECT_2.DATA_2]",
     "CONDITIONAL_4_NAME": "[Conditional.CONDITIONAL_1_NAME] || [Conditional.CONDITIONAL_2_NAME]",
     ...
 }
```

Data is supplied by data suppliers or constants from the `constants.json` file. Data suppliers are set up in the construction of an auto program, as a map from ids to double suppliers. These ids can be referenced in conditionals with the format `[ID]`. Previously defined conditionals can also be reference in the same way, returning 1 if they are `True` and 0 otherwise.

Two conditionals are always defined by default: `TRUE` which always returns `True` and `FALSE` which always returns `False`.

Outputs from data suppliers and constants can have basic arithmatic applied to them, both by integers and by other data supplier outputs. Addition (+), subtraction (-), multiplication (*), division (/), integer division (//), modulus (%), and exponents (^) can all be used. Values can be compared using ==, >, >=, <, <=, and !=. The results of these comparisons can be further processed using `or` (||) and `and` (&&).