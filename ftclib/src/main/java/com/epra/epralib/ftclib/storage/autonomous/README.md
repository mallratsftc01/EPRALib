## Overview

The `storage.autonomous` subgroup contains the `AutoProgram` class, A very powerful class that can represent an entire autonomous program. An auto program is based on a directory of `JSON`s which represent individual movements, program structure, and conditionals that control flow.

An example of an auto directory's structure:

```
auto
├ program.json
├ conditionals.json
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
    "comment": "Example",
    "endCondition": "CONDITIONAL_1",
    "driveTrainModule": {
      "targetX": 0.0,
        "targetY": 0.0,
        "targetAngle": 0.0,
        "posTolerance": 0.5,
        "angleTolerance": 0.5,
        "maxPower": 1.0, 
        "usePrecision": true
    },
    "motorControllerModules": [
        {
            "id": "MOTOR_1",
            "target": 100,
            "tolerance": 0.5,
            "maxPower": 1.0,
            "power": 0.0
        }
    ]
}
```

The DTAM controls how the `DriveTrain` moves, using the `posPIDMecanumDrive` method in conjunction with data from `Odometry` and `MultiIMU`. It will drive the robot towards the `targetX`, `targetY` position and rotate to the `targetAngle`. If `usePrecision` is true, the `posTolerance` and `angleTolerance` will control how closely the robot must reach the target position and angle, respectively. Otherwise, the robot will simply move towards the target position but not try to hit it exactly. This movement mode is useful when following a general path. The `maxPower` controls how much power can be sent to the drive train's motors. If set to 0, the robot will not move.

Each MCAM gives instructions to a single `MotorController`. They should **not** be used for drive train motors, as that can interfere with the instructions from the DTAM. Just like the DTAM, MCAMs have two main "modes". If the `tolerance` is set to -1.0, the power for the motor can be set directly. This is useful when you don't need the motor to reach a certain point, only reach a certain speed, such as spinning up a shooter wheel. When the `tolerance` is above 0, it is used to determine how close the motor should try to get to the `target`. The `target` is in motor-specific ticks, and is reached via PID loop. The `maxPower` caps the amount of power that can be sent to the motor.

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

Data is supplied by data suppliers. These are set up in the construction of an auto program, as a map from ids to double suppliers. These ids can be referenced in conditionals with the format `[ID]`. Previously defined conditionals can also be reference in the same way, returning 1 if they are `True` and 0 otherwise.

Two conditionals are always defined by default: `TRUE` which always returns `True` and `FALSE` which always returns `False`.

Outputs from data suppliers can have bany basic arithmatic applied to them, both by integers and by other data supplier outputs. Addition (+), subtraction (-), multiplication (*), division (/), integer division (//), modulus (%), and exponents (^) can all be used. Values can be compared using ==, >, >=, <, <=, and !=. The results of these comparisons can be further processed using or (||) and and (&&).