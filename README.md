# rob – virtual R.O.B. for MAME

A MAME Lua plugin for the NES / Famicom games that use R.O.B. (the Robotic Operating Buddy, sold
in Japan as the Famicom Robot). It watches the picture for R.O.B.'s control frames, decodes them,
and animates a shaded 3D R.O.B. with the game's accessories in a corner of the screen. When the
game needs input back from the robot's accessories, the plugin presses the PLAYER 2 buttons for it.

## Install

Copy this folder to `<mame>/plugins/rob/` (it contains `init.lua`, `plugin.json` and six label
textures, `rob_sticker.ppm`, `famicom_sticker.ppm`, `spinner_label.ppm`, `switch_panel.ppm`,
`gyro_label.ppm` and `block_label.ppm`, then start a R.O.B.
game with the plugin, for example:

    mame nes -cart stackup -plugin rob
    mame famicom -cart gyromite -plugin rob

(or enable "rob" in *Plugins* from the MAME menu / `plugin.ini`). Needs a MAME recent enough
to have `screen:draw_box` in pixel coordinates (0.227+) and `screen:pixel`.

## How it works

* **Control frames** – a uniformly green frame = 1, a uniformly black frame = 0, anything
  else is ordinary gameplay. The plugin samples a 12x10 grid every frame.
* **Protocol** (nesdev.org/wiki/R.O.B.) – one bit per vsync, command = `000101w1x1y1z`.

  | wxyz | command | wxyz | command |
  |------|---------|------|---------|
  | 0101 | Up 2    | 0100 | Turn left  |
  | 1100 | Up 1    | 1000 | Turn right |
  | 0010 | Down 1  | 0110 | Close arms |
  | 1101 | Down 2  | 1010 | Open arms  |
  | 1001 | LED on  | 0001 | Reset      |

  A 25/30 Hz alternating stream (the aiming aid) makes the LED blink.
* **Robot model** – 5 body stops (60° apart), 6 arm heights, open/close claws. Motors run at a
  fixed speed in emulated time (two stops take about 3 seconds). The hands and arms are solid:
  whatever they run into is knocked over.
* **Accessories** – the game being played picks the accessories:
  * **Gyromite** – two gyros, two holders, a spinner and red/blue button trays. A gyro spins up
    on the spinner's tower. A gyro pushes a tray's button down by its weight:
    set free on a tray it only stays up while it spins (a slow one topples off); held in the
    claws and lowered onto the tray it presses the button spinning or not. A gyro let go on top
    of another knocks it over. Toppled gyros are carried back to free holders.
  * **Stack-Up** – five blocks on five trays. The hands and arms knock blocks off a stack, and
    knocked blocks are carried back to their tray once the hands are out of the way.
* **Player 2 interface** – at start the plugin scans the ioport list for player-2 fields.
  Accessories that report back to the game press them: in Gyromite the blue tray button presses
  controller 2 A and the red one controller 2 B (fields `P2 A` / `P2 B`). Buttons are driven with
  `field:set_value(1)` and released with `field:clear_value()`.

## Tuning

Everything is in the `cfg` table at the top of `init.lua`:

* panel position and size, and the size of the robot in it
* green/black detection thresholds
* motor speeds: `body_step_time`, `arm_step_time`, `claw_time`, `tray_step_time`
* accessory physics: `disc_fall_speed` (Stack-Up blocks); `spinup_time`, `dock_stop_time`,
  `spin_duration` and `balance_spin` (Gyromite gyros)
* the Gyromite accessory layout (`cfg.stations`, stop 0 = fully right ... stop 4 = fully left)

Colour scheme: `cfg.variant = "auto"` follows the emulated machine. `"us"` gives the NES R.O.B.:
light grey, with mid-grey arms and a darker grey base band. `"famicom"` gives the Famicom Robot:
white, with maroon arms, base band and claws. Both have a smoked-black visor and hands.

## Caveats

* Tested against MAME 0.283+ (`nes` and `famicom`, Gyromite and Stack-Up). If nothing decodes,
  loosen `flash_min` / `match_fraction`.
* The accessory layouts and which tray presses which P2 button are a best guess; adjust
  `cfg.stations` and the button patterns if a game disagrees.
