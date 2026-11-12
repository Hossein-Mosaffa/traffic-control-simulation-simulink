# Fuzzy Logic Car-Following Controller

A Simulink model in which a follower car keeps a fixed distance behind a lead car. The follower is steered by a Mamdani fuzzy logic controller. It was built in January 2018 as a Computational Intelligence course project, using MATLAB R2017b, Simulink and the Fuzzy Logic Toolbox.

The goal is simple. Whatever the lead car does, the follower should stay 25 units behind it. The controller sees two numbers, the distance error and the speed difference between the cars, and returns a small correction. That correction is added to the lead car's speed, and the sum becomes the follower's speed command.

A short English write-up with figures is in [Project_Summary_Report.pdf](Project_Summary_Report.pdf).

## What is in this repository

| Path | Description |
| --- | --- |
| `T_C.slx` | The Simulink model |
| `Rule.fis` | The fuzzy inference system: two inputs, one output, 25 rules |
| `Project_Summary_Report.pdf` | English summary of the project, with figures |

## Requirements

You need MATLAB with Simulink and the Fuzzy Logic Toolbox. The model was saved in R2017b. The `VR Visualization` block also needs Simulink 3D Animation and a world file called `octavia_scene_lchg_2cars.wrl`, which is not part of this repository. The block only draws the cars, so if you do not have these you can delete it or comment it out and the control loop is unaffected.

## Running the model

Open `T_C.slx` in MATLAB.

The controller block refers to its rule base by the name `Rule`. Keep `Rule.fis` in the folder you run from. If Simulink cannot find it, load it into the workspace before starting the simulation:

```matlab
Rule = readfis('Rule.fis');
```

Then pick a lead-car profile (see below) and run. The stop time is 600 and the solver is variable-step, chosen automatically. The scope called `Speed` shows the selected lead command, the lead car's speed and the follower's speed. `Scope2` shows all three profiles side by side.

To look at the fuzzy system on its own:

```matlab
fis = readfis('Rule.fis');
plotmf(fis, 'input', 1)
plotmf(fis, 'input', 2)
plotmf(fis, 'output', 1)
gensurf(fis)
```

## How the Simulink model works

The signal flow is a feedback loop around the follower car.

```
input command -> switches -> Front_Car ---------------+--> (+) -> Follower_Car --+--> follower speed
                                |                      ^                         |
                                v                      | correction              |
                      Subsystem (errors) <-------------+-------------------------+
                                |                      |
                                +--> gains x0.1 --> Fuzzy Logic Controller
```

The main blocks and their settings, as saved in the model:

| Block | Settings | Role |
| --- | --- | --- |
| `input command` | Subsystem with three outputs | Generates the three lead-car speed profiles |
| `Manual Switch`, `Manual Switch2` | Chosen by hand | Select which profile drives the lead car |
| `Front_Car` | Transfer Fcn `1/(0.5s^2+0.5s+1)`, Transport Delay 0.1 s | Lead car, turns a command into a speed |
| `Follower_Car` | Same as the lead car | Follower car |
| `Subsystem` | Integrators (initial values 5 and 0), constant 25 | Computes the distance and speed errors |
| `Gain`, `Gain2` | 0.1 each | Scale the errors toward the controller's input range |
| `Fuzzy Logic Controller` | FIS `Rule`, 101 output samples | Maps the two errors to a correction |
| `Add` | Two inputs, both added | Lead car speed plus correction, which is the follower's command |
| `VR Visualization` | VR Sink with the `.wrl` world | 3D view of both cars, display only |

Both cars use the same second-order transfer function. Its natural frequency is about 1.41 rad/s and its damping ratio about 0.35, so a speed command is followed with roughly 30 percent overshoot before it settles. The 0.1 s transport delay stands in for driver reaction time and vehicle lag, as the original report explains.

The error subsystem has two inputs, the lead car's speed and the follower's speed. It integrates each speed to get a position. The lead integrator starts at 5 and the follower's at 0, so the cars begin 5 units apart. It then produces four outputs.

| Output | Formula |
| --- | --- |
| `D_Distance` | (lead position - follower position) - 25 |
| `D_Speed` | lead speed - follower speed |
| `Front_Pos` | lead position, used for the 3D view |
| `folower_x` | follower position, used for the 3D view (spelled this way in the model) |

A positive `D_Distance` means the follower has fallen more than 25 units behind. A negative value means it is closer than 25 and in danger of hitting the lead car. A positive `D_Speed` means the lead car is pulling away. These two numbers, each multiplied by 0.1, are the inputs of the fuzzy controller.

The `VR Visualization` block receives both positions, each padded with the constant `[0.2 0]` to form a three-element position vector, plus fixed color vectors.

## Lead-car profiles

The `input command` subsystem builds three speed commands by adding Step, Ramp and Sine Wave blocks. The model does not state physical units, so the values are plain numbers and times are in seconds.

| Profile | Blocks | Resulting command |
| --- | --- | --- |
| 1, `Out1` | Steps to 30 at t=0, then +20 at 100, -10 at 200, -5 at 300, plus a sine of amplitude 5 at 0.1 rad/s | Levels of 30, 50, 40 and 35 with a sine ripple on top |
| 2, `Out2` | Ramps of slope +1 from t=0 and -1 from t=200, steps of +10 at 100, -10 at 300 and -20 at 400 | Climbs to 210 by t=200, then drops to 200 and 180 |
| 3, `Out3` | Ramps of slope +2 from t=0, -2 from t=100, -1 from t=150 and +1 from t=200 | Climbs to 200 by t=100, holds, falls to 150 by t=200 and stays |

The two manual switches work in sequence. `Manual Switch` chooses between profile 1 (upper input) and profile 2 (lower input). `Manual Switch2` chooses between that result (upper input) and profile 3 (lower input). To run profile 3, flip `Manual Switch2` to its lower input. Double-click a switch to flip it, even while the simulation is running.

## How `Rule.fis` works

The `.fis` file is plain text with five sections. `[System]` holds the global settings:

```
Type='mamdani'
NumInputs=2   NumOutputs=1   NumRules=25
AndMethod='min'   OrMethod='max'
ImpMethod='min'   AggMethod='max'
DefuzzMethod='centroid'
```

For each rule, the controller takes the smaller of the two input memberships (min for AND), clips the output set at that level (min for implication), combines all the clipped sets by taking the maximum (max for aggregation), and finds the centroid of the result. In the model the centroid is computed over 101 sample points.

The two inputs and the output each span -1 to 1. Their membership functions are defined by type and parameters, and the sets are listed in order, so the number in the rule list is the position in this list.

| Diff-Position (distance error) | Type | Parameters |
| --- | --- | --- |
| 1 `N` (negative) | trapmf | -100 -100 -0.33 0 |
| 2 `Z` (zero) | trimf | -0.33 0 0.33 |
| 3 `PS` (positive small) | trimf | 0 0.33 0.66 |
| 4 `PM` (positive medium) | trimf | 0.33 0.66 1 |
| 5 `PB` (positive big) | trapmf | 0.66 1 100 100 |

| Diff-Velocity (speed error) | Type | Parameters |
| --- | --- | --- |
| 1 `NB` | trapmf | -100 -100 -1 -0.5 |
| 2 `NS` | trimf | -1 -0.5 0 |
| 3 `Z` | trimf | -0.5 0 0.5 |
| 4 `PS` | trimf | 0 0.5 1 |
| 5 `PB` | trapmf | 0.5 1 100 100 |

| signal-improvment (output) | Type | Parameters |
| --- | --- | --- |
| 1 `NB` | trapmf | -100 -100 -1 -0.66 |
| 2 `NM` | trimf | -1 -0.6666 -0.3334 |
| 3 `NS` | trimf | -0.6666 -0.3334 0 |
| 4 `Z` | trimf | -0.3334 0 0.3334 |
| 5 `PS` | trimf | 0 0.3334 0.6666 |
| 6 `PM` | trimf | 0.3334 0.6666 1 |
| 7 `PB` | trapmf | 0.66 1 100 100 |

The `-100` and `100` values in the trapezoids are a common trick for shoulder sets. They push the flat edge far outside the range, so the set stays at full membership all the way to the end of the interval.

Each line in the `[Rules]` section reads like this:

```
2 3, 4 (1) : 1
```

That means distance error is set 2 (`Z`) and speed error is set 3 (`Z`), so the output is set 4 (`Z`). The `(1)` is the rule weight and the final `1` means the two conditions are joined with AND. The 25 rules form a full 5 by 5 table:

| Diff-Position \ Diff-Velocity | NB | NS | Z | PS | PB |
| --- | --- | --- | --- | --- | --- |
| N | NB | NB | NB | NB | NB |
| Z | NM | NS | Z | PS | PM |
| PS | NS | Z | PS | PM | PB |
| PM | Z | PS | PM | PB | PB |
| PB | PS | PM | PB | PB | PB |

The first row is the safety rule. If the follower is closer than 25 units, the output is NB whatever the speed difference, so the controller always brakes as hard as it can. In the other four rows the output set index rises by one for each step up in either error and stops at PB, so the correction grows as the follower falls further behind or as the lead car pulls away faster.

## Things to know

The saved model uses 0.1 for both input gains. A model screenshot in the original course slides shows 1, so the slides probably come from an earlier version.

The controller output stays between -1 and 1 and is added to the lead car's speed, so it is a small correction next to lead speeds like the 30 to 210 in the profiles. No closed-loop results are saved in this repository, so how tightly the gap is held for each profile is something to check by running the simulation.

The original Persian course report names a few sets slightly differently from `Rule.fis`. It calls the fourth distance-error set `MS` instead of `PM`, and it lists the output sets `NS` and `NM` in the opposite order. The `.fis` file is what the model actually uses, and this README follows it. The output variable is spelled `signal-improvment` in the file.

## Authors

Hossein Mosaffa, Seyed Hesam Salehnejad, and Ali Nikhalat Jahromi, for the Computational Intelligence course taught by Dr. Abdollahi at Amirkabir University of Technology (Tehran Polytechnic).

[Hossein Mosaffa's personal website](https://hossein-mosaffa.github.io)
