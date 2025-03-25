# 2025-Reefscape

Spartronics 4915's code for the 2025 FRC season reefscape!

# Control Map

## Driver controller

| input   | action |
| -------- | ------- |
| Left Stick |  Drive |
| Push Left Stick |  Reset field relative heading |
| Right Stick | Rotate |
| Push Right Stick | *None* |
| A | Set field relative heading |
| B | Toggle field relative |
| X | *None* |
| Y | *None* |
| Left Trigger | X-brake swerve |
| Right Trigger | Reef / coral station auto orient |
| Left Bumper | Auto-align to left branch |
| Right Bumper | Auto-align to right branch |
| D-pad Up | Nudge forwards |
| D-pad Down | Nudge backwards |
| D-pad Left | Nudge left |
| D-pad Right | Nudge right |
| Windows | *None* |
| Menu | Auto-align to middle of reef |

## Operator controller
| input   | action |
| -------- | ------- |
| Left Stick | *None* |
| Push Left Stick | High algae preset |
| Right Stick | *None* |
| Push Right Stick | Low algae preset |
| Release Left/Right Stick | Algae arm preset |
| A | L1 scoring preset |
| B | L2 scoring preset |
| X | L3 scoring preset |
| Y | L4 scoring preset |
| Left Trigger | Stow |
| Right Trigger | Score |
| Left Bumper | Retract/ease winch (toggles mode on press) |
| Right Bumper | Engage/retract climber (toggles mode on press) & arm climb preset |
| D-pad Up | (Manual) Elevator up |
| D-pad Down | (Manual) Elevator down |
| D-pad Left | (Manual) Arm CCW |
| D-pad Right | (Manual) Arm CW |
| Windows | Unstuck coral |
| Menu | Intake |

## Debug controller
this is a controller that shouldn't be used during competition matches
this controller will have tools for debugging
* manual control for preset tuning (with more control than manual mode with the operator controller)

| input   | action |
| -------- | ------- |
| B | Force vision to use MegaTag 1 |
| X | Ignore vision readings |

# Bling/Driver Communication

Before match start: Solid purple if can't see 2 tags, will do a Spartronics themed light show once it can.

If a MegaTag1 reading is needed it'll be a rapidly flashing cyan, overriding all other bling, other than autos.

Upon end of match, the robot will flash red for 5 seconds, then play a fun light show.

Reef Zone:
| Color | Meaning |
| --- | --- |
| White |  Closest reef side is the closest to the drivers |
| Orange | Closest reef side is the 2nd closest to the drivers |
| Green | Closest reef side is the 2nd furthest to the drivers |
| Purple | Closest reef side is the furthest from drivers |

Coral Station:
| Color | Meaning |
| --- | --- |
| Rainbow | Ready to go |
| Green Flashing | Awaiting Coral |
| Yellow Flashing | Mechanisms need to move |

Barge:
| Color | Meaning |
| --- | --- |
| Rainbow | CLIMB CLIMB CLIMB GO GO GO |
| Blue | Move to the right |
| Red | Move to the left |