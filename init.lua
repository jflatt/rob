-- license:BSD-3-Clause
-- Virtual R.O.B. (Robotic Operating Buddy) for MAME's NES driver, built for Gyromite.
--
-- What it does, every frame:
--   1. Samples the emulated screen. A uniformly GREEN frame is a "1" bit, a uniformly
--      BLACK frame is a "0" bit, anything else is normal gameplay.
--   2. Watches the stream of bits for the R.O.B. command frame  000101w1x1y1z  (13 vsyncs)
--      and decodes wxyz into one of the ten known robot commands.
--   3. Runs a small physical simulation of the robot (body turn, arm height, claws,
--      two gyros, spinner, red/blue trays).
--   4. Draws the robot as a software-rendered, shaded 3D model on top of the game picture.
--   5. When a gyro presses a tray (a spinning one set on it, or any held down on it by the claws), holds the matching PLAYER 2 button down
--      through MAME's ioport system, just like the real tray lever does on the real hardware.
--
-- Protocol reference: https://www.nesdev.org/wiki/R.O.B.
--   Black ($0F) = 0 bit, bright green ($2A) = 1 bit, one bit per vertical sync.

local exports = {
	name = "rob",
	version = "0.1.0",
	description = "Virtual 3D R.O.B.",
	license = "BSD-3-Clause",
	author = { name = "jflatt" } }

local rob = exports

function rob.startplugin()

	---------------------------------------------------------------------------
	-- CONFIGURATION  (everything you might want to tune lives here)
	---------------------------------------------------------------------------
	local cfg = {
		-- on-screen panel, in emulated-screen pixels
		panel_w = 112, panel_h = 120,
		panel_anchor = "bottom-right",        -- bottom-right, bottom-left, top-right, top-left
		panel_margin = 0,                     -- gap to the screen edges (0 = right in the corner)
		outline = false,                      -- dark outline on every face (costs many extra draw calls)
		focal_scale = 1.0,                    -- size of the robot (1.0 fills a 112 x 120 panel)
		mirror_lr = false,                    -- false: TURN LEFT = R.O.B.'s left (screen right when he faces you)
		variant = "auto",                     -- "auto" (follow the emulated machine), "us" (NES grey) or "famicom"
		game = "auto",                        -- "auto" (detect the cartridge), "gyromite", "stackup", "xmas14" or "none"
		fallback_game = "none",               -- when the cartridge is not recognised: "none" = just the robot, no attachments
		detect_every = 120,                   -- re-check machine + cartridge every N frames (catches late loads)


		-- control-frame detection
		sample_cols = 12, sample_rows = 10,   -- grid of sample points over the screen
		match_fraction = 0.90,                -- share of samples that must agree
		flash_min = 90,                       -- any colour counts as a "1" flash once its brightest channel reaches this
		black_max = 28,                       -- every channel below this = black (a "0" frame)


		-- robot physics, expressed in seconds
		body_step_time = 1.5,                -- one body turn step (left / right): two stops take 3 s
		arm_step_time  = 1.5,                -- one arm height step (up / down); Gyromite moves two at a time (3 s)
		claw_time      = 0.5,                -- open <-> close
		disc_fall_speed = 0.12,               -- Stack-Up disks fall with this share of real gravity (1 = real; lower = slower, easier to follow)
		spinup_time    = 1.2,                 -- time on the spinner to reach full spin
		dock_stop_time = 0.5,                 -- a spinning gyro set in a holder is braked to a stop in this time
		spin_duration  = 150.0,               -- full spin -> stopped once off the spinner (a real gyro spins for a couple of minutes)
		balance_spin   = 0.15,                -- spin a gyro set free on a red / blue button needs to stay up (less: it topples off)
		tray_step_time = 1.5,                 -- a red / blue button sinks (and springs back) one arm step in this time
		frame_rate     = 60.0,

		-- Gyromite layout, left to right as you see it when R.O.B. faces you:
		-- 2 gyro holders, blue button (controller A), red button (controller B), spinner motor.
		-- Stop 0 = fully to R.O.B.'s right (your left) ... stop 4 = fully to his left (your right).
		-- Reverse the order here if you want to read it from R.O.B.'s point of view instead.
		stations = {
			[0] = { role = "dock" },
			[1] = { role = "dock" },
			[2] = { role = "tray", color = "blue" },
			[3] = { role = "tray", color = "red"  },
			[4] = { role = "spinner" },
		},
	}

	-- wxyz -> command.   (nesdev.org/wiki/R.O.B.)
	local COMMANDS = {
		["0101"] = { name = "UP 2",         kind = "up",    n = 2 },
		["1100"] = { name = "UP 1",         kind = "up",    n = 1 },
		["0010"] = { name = "DOWN 1",       kind = "down",  n = 1 },
		["1101"] = { name = "DOWN 2",       kind = "down",  n = 2 },
		["0100"] = { name = "TURN LEFT",    kind = "left"  },
		["1000"] = { name = "TURN RIGHT",   kind = "right" },
		["0110"] = { name = "CLOSE ARMS",   kind = "close" },
		["1010"] = { name = "OPEN ARMS",    kind = "open"  },
		["1001"] = { name = "LED ON",       kind = "led"   },
		["0001"] = { name = "RESET",        kind = "reset" },
	}

	---------------------------------------------------------------------------
	-- small helpers
	---------------------------------------------------------------------------
	local floor, ceil, sin, cos, sqrt, abs, max, min, huge =
		math.floor, math.ceil, math.sin, math.cos, math.sqrt, math.abs, math.max, math.min, math.huge
	local RAD = math.pi / 180

	local function clamp(v, lo, hi) if v < lo then return lo elseif v > hi then return hi end return v end

	local function approach(cur, target, step)
		if cur < target then return min(cur + step, target) end
		return max(cur - step, target)
	end

	local function argb(a, r, g, b)
		return (floor(a) << 24) | (floor(clamp(r, 0, 255)) << 16) | (floor(clamp(g, 0, 255)) << 8) | floor(clamp(b, 0, 255))
	end

	-- Texture images (binary PPM, "P6") shipped next to this file: the base sticker and the
	-- spinner's label. A missing image just leaves that decal as a flat colour.
	local PLUGIN_DIR = (function()
		local src = debug.getinfo(1, "S").source or ""
		if src:sub(1, 1) == "@" then return src:sub(2):match("^(.*)[/\\]") end
	end)()
	local function load_ppm(name)
		if not PLUGIN_DIR then return nil end
		local f = io.open(PLUGIN_DIR .. "/" .. name, "rb")
		if not f then return nil end
		local data = f:read("a")
		f:close()
		local w, h, mx, pos = data:match("^P6%s+(%d+)%s+(%d+)%s+(%d+)%s()")
		if not w then return nil end
		w, h = tonumber(w), tonumber(h)
		local px = data:sub(pos, pos + w * h * 3 - 1)
		if #px < w * h * 3 then return nil end
		local sr, sg, sb, n = 0, 0, 0, 0
		for i = 1, #px, 3 * 37 do
			local r, g, b = px:byte(i, i + 2)
			sr, sg, sb, n = sr + r, sg + g, sb + b, n + 1
		end
		return { w = w, h = h, px = px, avg = { sr / n, sg / n, sb / n } }
	end
	local TEX = {}

	---------------------------------------------------------------------------
	-- machine / screen access (resolved lazily so the plugin loads without a machine)
	---------------------------------------------------------------------------
	local scr = nil
	local pixel_mode = nil        -- "pixel" or "pixels"
	local frame_no = 0

	local function find_screen()
		local ok, screens = pcall(function() return manager.machine.screens end)
		if not ok or not screens then return nil end
		for _, s in pairs(screens) do return s end
		return nil
	end

	---------------------------------------------------------------------------
	-- 1. CONTROL-FRAME DETECTION
	---------------------------------------------------------------------------
	-- screen.width / screen.height are properties in newer MAME, methods in older builds
	local function screen_size()
		local w, h = scr.width, scr.height
		if type(w) == "function" then w = scr:width() end
		if type(h) == "function" then h = scr:height() end
		return w, h
	end

	-- drawing wrapper, in screen pixel coordinates.
	-- NB: MAME's signature is draw_box(left, top, right, bottom, LINE, FILL): the outline colour comes
	-- first. This wrapper takes (fill, line) and passes them in MAME's order. A line colour of 0 is
	-- fully transparent, i.e. no outline.
	local function dbox(x1, y1, x2, y2, fill, line)
		scr:draw_box(x1, y1, x2, y2, line, fill)
	end

	local sample_points = nil

	local function build_samples(w, h)
		sample_points = {}
		for j = 0, cfg.sample_rows - 1 do
			for i = 0, cfg.sample_cols - 1 do
				local x = floor((i + 0.5) * w / cfg.sample_cols)
				local y = floor((j + 0.5) * h / cfg.sample_rows)
				sample_points[#sample_points + 1] = { x, y }
			end
		end
	end

	local function read_pixel_fn(w, h)
		-- Prefer screen:pixel(); fall back to a pixels() snapshot (32-bit ARGB, little endian).
		if pixel_mode == nil then
			local ok = pcall(function() return scr:pixel(0, 0) end)
			pixel_mode = ok and "pixel" or "pixels"
		end
		if pixel_mode == "pixel" then
			return function(x, y) return scr:pixel(x, y) end
		end
		local buf = scr:pixels()
		return function(x, y)
			local p = (y * w + x) * 4 + 1
			if p + 3 > #buf then return 0 end
			return (string.unpack("<I4", buf, p))
		end
	end

	-- Returns two classifications of the current frame:
	--   bit : 1 (full-screen flash), 0 (black frame) or nil (ordinary picture) -- for command frames
	--   lit : 1 if a bright patch shows on an otherwise black screen, 0 for a black frame, else nil.
	--         Games aim R.O.B. with a flashing patch rather than the whole screen (Gyromite's TEST
	--         mode flashes a green box covering ~40% of the screen at 30 Hz), so the LED blink
	--         detector uses this looser test.
	local function classify_frame()
		local w, h = screen_size()
		if not sample_points or sample_points.w ~= w or sample_points.h ~= h then
			build_samples(w, h)
			sample_points.w, sample_points.h = w, h
		end
		local getpix = read_pixel_fn(w, h)
		local greens, blacks = 0, 0
		local n = #sample_points
		local need = n * cfg.match_fraction
		for i = 1, n do
			local sp = sample_points[i]
			local p = getpix(sp[1], sp[2])
			local r, g, b = (p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF
			if r < cfg.black_max and g < cfg.black_max and b < cfg.black_max then
				blacks = blacks + 1
			elseif max(r, g, b) >= cfg.flash_min then       -- any bright colour is a "1"
				greens = greens + 1
			end
		end
		local bit = (greens >= need and 1) or (blacks >= need and 0) or nil
		local lit = (blacks >= need and 0)
			or ((greens >= n * 0.2 and greens + blacks >= n * 0.95) and 1) or nil
		return bit, lit
	end

	---------------------------------------------------------------------------
	-- 2. PROTOCOL DECODER   000101w1x1y1z
	---------------------------------------------------------------------------
	local hist = {}                -- sliding window of the last 13 binary frames
	local last_decode_frame = -100
	local alt_run = 0              -- consecutive alternating bits (LED aim blink stream)
	local last_bit = nil
	local blink_until = 0
	local last_cmd = { name = "-", code = "----", frame = -1000 }
	local on_command               -- forward declaration (robot simulation)

	local function decode_push(bit, lit)
		-- aim-assist blink stream (25/30 Hz flashes, full-screen or a flashing patch): the LED blinks
		-- while light and dark frames keep alternating
		if lit == nil then alt_run, last_bit = 0, nil
		else
			if last_bit ~= nil and lit ~= last_bit then alt_run = alt_run + 1 else alt_run = 0 end
			last_bit = lit
			if alt_run >= 10 then blink_until = frame_no + 6 end
		end

		if bit == nil then
			hist = {}
			return
		end
		hist[#hist + 1] = bit
		if #hist > 13 then table.remove(hist, 1) end

		if #hist < 13 then return end
		local b = hist
		if b[1] == 0 and b[2] == 0 and b[3] == 0 and b[4] == 1 and b[5] == 0 and b[6] == 1
			and b[8] == 1 and b[10] == 1 and b[12] == 1
			and frame_no - last_decode_frame >= 13 then
			local code = string.format("%d%d%d%d", b[7], b[9], b[11], b[13])
			last_decode_frame = frame_no
			local cmd = COMMANDS[code]
			last_cmd = { name = cmd and cmd.name or "NOP", code = code, frame = frame_no }
			if cmd and on_command then on_command(cmd) end
		end
	end

	---------------------------------------------------------------------------
	-- 3. ROBOT SIMULATION
	---------------------------------------------------------------------------
	-- Arm rig (plan view, right arm, claws open), from photos of real units checked against the
	-- patent's top view: the arm pivots at ARM_PIV inside the shoulder housing's lobe; the grip point
	-- (the claw's notch / the grip's inner face) is at ARM_APEX, 7.4 ahead of the mast, and the
	-- grippers open to 4.4 apart. Closing swings the arm about the pivot until they meet.
	local ARM_PIV_X, ARM_PIV_Z = 3.9, 0.4
	local ARM_APEX_X, ARM_APEX_Z = 2.2, 7.4
	local GRIP_OPEN = 2 * ARM_APEX_X
	-- Each arm is one rigid piece from the shoulder pivot to a second pivot on the hand (the hole in
	-- the arm's rounded end, HP_DX / HP_DZ from the notch); the hand hangs on that pivot and stays
	-- parallel, so the notch keeps a fixed offset from the hand pivot.
	local HP_DX, HP_DZ = 2.75, -0.1
	local HP_A, HP_B = ARM_APEX_X + HP_DX - ARM_PIV_X, ARM_APEX_Z + HP_DZ - ARM_PIV_Z
	local HP_R, HP_PHI = sqrt(HP_A * HP_A + HP_B * HP_B), math.atan(HP_B, HP_A)
	local function arm_swing(apex_x)               -- arm swing (0 = open) that puts the notch at x = apex_x
		local c = (apex_x + HP_DX - ARM_PIV_X) / HP_R
		if c < -1 then c = -1 elseif c > 1 then c = 1 end
		return HP_PHI - math.acos(c)
	end
	local function grip_z(apex_x)                -- how far ahead of the mast the notch is for a given notch x
		local d = arm_swing(apex_x)
		return ARM_PIV_Z - HP_A * sin(d) + HP_B * cos(d) - HP_DZ
	end
	local HOLD_R = grip_z(0.275)       -- distance from the axis to a held / docked item (set per game)
	local R = {}                       -- robot state
	local gyros = {}
	local tray_down = { red = 0, blue = 0 }

	local game = "none"                -- gyromite | stackup | xmas14 | none   (set by detection / cfg.game)
	local machine_kind = "nes"         -- nes | famicom                  (set by detection / cfg.variant)
	local configured = false
	local STATIONS = cfg.stations      -- accessory layout for the current game
	local stacks = {}                  -- Stack-Up: stacks[stop] = { disk, disk, ... } bottom -> top
	local falling = {}                 -- Stack-Up: groups of disks just released, dropping onto a stack
	local knocked = {}                 -- Stack-Up: disks knocked off a stack by the arms (tumbling / resting / carried back)
	-- Arm travel: 6 stops, 1.078 apart. The arms are horizontal and ride with the shoulder housing,
	-- which is hollow underneath: at the lowest stop it sits on the turntable (over the collar),
	-- hands at 4.16; at the top stop (power-on / RESET) it is just under the head, hands at 9.55.
	local ARM_STEP = 1.078             -- one arm height step (5 steps: housing on the turntable at 3.2 .. its
	                                   -- top position, bottom at 8.59 just under the head -- patent front view)
	local DISC_H = ARM_STEP            -- height of one Stack-Up block = one arm step (the hands meet block k at stop k)
	local STACK_SEAT = 3.621           -- Stack-Up: bottom block rests in its tray here (level with the lowest hand)
	local HAND_Y0 = STACK_SEAT + DISC_H / 2   -- hand height at stop 0 (4.16: housing resting on the turntable)
	-- the gyro's leg (spindle below the disc, tip to disc bottom): it sinks into the spinner's tower,
	-- whose sleeve is about as long
	local GYRO_TIPL = 1.99
	-- how much higher a gyro stands on a fully pressed red / blue button than in a holder (see TRAY_TRAVEL)
	local BTN_LIFT = 0.29
	-- gyro spindle length above the disc. A gyro is a little under four arm steps tall: one carried with
	-- the arms all the way up just clears (by 0.1) the spindle of a gyro on a pressed button (and those
	-- in the holders, gripped at stop 1 four steps lower, by a little more); carried lower it runs into them.
	local SPINDLE_LEN = 4 * ARM_STEP - 0.22 - GYRO_TIPL - BTN_LIFT - 0.1
	-- Height of a gyro's disc standing in a holder / on a pressed button: exactly as high as it hangs from
	-- the claws at stop 1 (Gyromite's low position), so the claws close just under the spindle's cap and
	-- the hands stay clear above the disc. (A hanging gyro's disc is 0.27 + 0.22 + SPINDLE_LEN below the
	-- hands -- see the held-gyro renderer.)
	local GYRO_SEAT = HAND_Y0 + ARM_STEP + 0.27 - (0.22 + SPINDLE_LEN)
	-- Gyromite: top of a gyro holder / button cap (pressed): the gyro's leg goes down inside the post, the
	-- disc standing 0.6 above it
	local G_SEAT = GYRO_SEAT - 0.6
	-- How far a red / blue button (and the gyro standing on it) goes down. The button (photos): a white post
	-- with a flat red / blue cap on top. A gyro stands on the cap on its tip (its leg never goes into the
	-- button). Lowered in the claws, at stop 3 it hangs 0.35 above the cap; it lands on it a little after
	-- and pushes it down with the arms, through stop 4, until the cap stops just above the post's foot
	-- shortly before stop 5 (the claws slide the last BTN_LIFT down the spindle). The disc of a gyro on a
	-- fully pressed button is BTN_LIFT above a holder's, TRAY_TRAVEL higher unpressed; the cap's top is
	-- GYRO_TIPL below the disc. (Pressed, the cap, 0.3 thick, sits just above the posts' 1.5 tall foot.)
	local TRAY_TRAVEL = 2 * ARM_STEP - 0.35 - BTN_LIFT
	-- The spinner seats its gyro two stops above the holders: Gyromite sets it down and takes it off at
	-- stop 3 (two below the top), the claws just under the spindle's cap as on a holder. The gyro's leg
	-- goes down inside the tower, whose top (seat_of(spinner) - 0.12) sits just under the disc.
	local SPIN_UP = 2 * ARM_STEP
	-- disc height of a gyro standing on station s (loc: its location there, for a pressed tray)
	local function seat_of(s, loc)
		local st = s and STATIONS[s]
		local y = GYRO_SEAT
		if st and st.role == "spinner" then y = y + SPIN_UP end
		if st and st.role == "tray" and st.color then y = y + BTN_LIFT + TRAY_TRAVEL * (1 - (R.press[st.color] or 0)) end
		return y
	end
	-- gravity in scene units per frame^2: 9.81 m/s^2, 1 unit = 1.85 cm, 60 frames per second
	local GRAVITY = 9.81 / 0.0185 / (60 * 60)
	local DISC_GRAVITY = GRAVITY * (cfg.disc_fall_speed or 1)
	local DISC_TSCALE = sqrt(cfg.disc_fall_speed or 1)    -- slows a tumbling disk's sideways / turning speed to match
	local gyro_lowest                  -- defined with the gyro geometry (used by the topple animation)
	local BOUNCE = 0.28                -- share of the landing speed a dropped gyro bounces back with
	local apply_palette                -- defined with the scene
	local BLOCK_COL = {
		red = { 208, 44, 44 }, blue = { 56, 100, 218 }, yellow = { 232, 196, 48 },
		green = { 52, 160, 72 }, white = { 236, 236, 240 },
	}
	local STACKUP_STATIONS = {}
	for i = 0, 4 do STACKUP_STATIONS[i] = { role = "pad" } end

	-- Per-game accessory set:
	--   gyromite : 2 gyros, 2 holders, blue/red buttons, spinner (cfg.stations)
	--   stackup  : 5 numbered trays, 5 coloured disks, all stacked on tray 3 (stop 2) at the start
	--              (top to bottom: red, white, blue, yellow, green)
	--   xmas14 / none (unrecognised cartridge) : the robot on its own, no discs / gyros / trays
	local function setup_items()
		HOLD_R = grip_z(((game == "stackup") and 2.16 or 0.42) / 2)   -- where the closed claws meet
		gyros = {}
		stacks = {}
		falling = {}
		knocked = {}
		for i = 0, 4 do stacks[i] = {} end
		if game == "gyromite" then
			STATIONS = cfg.stations
			gyros = {
				red  = { color = "2", station = 1, loc = "dock", spin = 0, phase = 0 },
				blue = { color = "1", station = 0, loc = "dock", spin = 0, phase = 0 },
			}
		elseif game == "stackup" then
			STATIONS = STACKUP_STATIONS
			for _, c in ipairs({ "green", "yellow", "blue", "white", "red" }) do
				stacks[2][#stacks[2] + 1] = { color = c }
			end
		else
			STATIONS = {}
		end
	end

	-- closed claws: Stack-Up hands meet the block; Gyromite claws stop on a gyro's spindle (0.42) or,
	-- with nothing between them, close right up (0.15)
	local function grip_closed()
		if game == "stackup" then return 2.16 end
		return (R.holding or R.gyro_in) and 0.42 or 0.15
	end
	-- gap between the gripping notches: closed .. fully open (notches 6.8 apart, as on the model)
	local function claw_gap() return grip_closed() + R.claw * (GRIP_OPEN - grip_closed()) end   -- gap between the grippers

	-- Initial state when the plugin starts: centred, arms all the way up, claws open, LED off.
	local function init_robot()
		R.stop, R.stop_t = 2, 2        -- body stop 0..4 (continuous angle follows)
		R.ang = 0                      -- body yaw in degrees, +ve = toward R.O.B.'s left
		R.h, R.h_t = 5, 5              -- arm height stop 0..5
		R.claw, R.claw_t = 1.0, 1.0    -- 1 = open, 0 = closed
		R.press = { red = 0, blue = 0 } -- red / blue button travel: 0 = up, 1 = pressed all the way down
		R.led = false
		R.holding = nil
		R.steps = {}                   -- queued target steps (used by RESET)
		R.claw_done = true
		setup_items()
	end
	-- Machine reset: R.O.B. stays where it is and runs its RESET routine (LED off, claws open, all the
	-- way right and up, then back to centre), exactly as the RESET command does. Anything it was
	-- holding drops when the claws open.
	local function reset_robot()
		R.steps = {}
		if on_command then on_command({ name = "RESET", kind = "reset", force = true }) end
	end

	local function station_of_body()
		local s = R.stop_t
		if abs(R.ang - (s - 2) * 60) < 6 then return s end
		return nil
	end

	-- Gyromite: is a gyro's spindle between the claws (could be grabbed here)?
	local function gyro_in_claws()
		if game ~= "gyromite" or R.holding then return false end
		local s = station_of_body()
		if not s then return false end
		local claw_y = HAND_Y0 + R.h * ARM_STEP + 0.12
		local gs = seat_of(s)
		if claw_y < gs + 0.22 - 0.1 or claw_y > gs + 0.22 + SPINDLE_LEN + 0.1 then return false end
		for _, g in pairs(gyros) do
			if g.loc ~= "held" and g.loc ~= "anim" and g.station == s then return true end
		end
		return false
	end

	local function try_grab()
		local s = station_of_body()
		if not s or R.holding then return end
		if game == "stackup" then
			-- Closing the hands around the disk at their level (0 = bottom disk) lifts that disk AND
			-- every disk stacked on top of it: they travel together as one group.
			local st = stacks[s]
			local lvl = floor(R.h + 0.5)
			if st and lvl >= 0 and lvl < #st then
				local group = {}
				for i = lvl + 1, #st do group[#group + 1] = st[i] end
				for i = #st, lvl + 1, -1 do st[i] = nil end
				R.holding = group
			end
			return
		end
		if game ~= "gyromite" then return end
		-- the claws (centred 0.12 above the hand) must be on the gyro's spindle
		local claw_y = HAND_Y0 + R.h * ARM_STEP + 0.12
		local gs = seat_of(s)
		if claw_y < gs + 0.22 - 0.1 or claw_y > gs + 0.22 + SPINDLE_LEN + 0.1 then return end
		for _, g in pairs(gyros) do
			if g.loc ~= "held" and g.loc ~= "anim" and g.station == s then
				g.loc = "held"
				g.falling = false
				R.holding = g
				return
			end
		end
	end

	local function try_release()
		local g = R.holding
		if not g then return end
		local s = station_of_body()
		if not s then return end
		if game == "stackup" then
			if stacks[s] then
				falling[#falling + 1] = { group = g, s = s, y = R.held_y0 or (STACK_SEAT + R.h * ARM_STEP), vy = 0 }
				R.holding = nil
			end
			return
		end
		-- let go over a station that already has a gyro: it drops onto that gyro's spindle, knocking it
		-- over (the falling gyro's domino check), and tumbles off to the floor itself; both are then
		-- carried back to free holders
		for _, o in pairs(gyros) do
			if o ~= g and o.loc ~= "held" and o.loc ~= "anim" and o.station == s then
				local a = (s - 2) * 60 * RAD
				g.station = -1                                    -- (no station: it falls onto anything)
				g.anim = { kind = "fall", x = HOLD_R * sin(a), y = (g.last_gb or seat_of(s)) - GYRO_TIPL, z = HOLD_R * cos(a),
					th = 0.06, om = 0.06, dir = a + 0.5, vy = 0.001, vu = 0.03, t = 0 }
				g.loc, g.falling = "anim", false
				R.holding = nil
				return
			end
		end
		local role = STATIONS[s] and STATIONS[s].role or "dock"
		g.station = s
		g.loc = role                   -- "tray", "spinner" or "dock"
		-- it drops from the height the hands let go of it (set by the renderer) and falls onto the station
		g.fy, g.fvy = g.last_gb or seat_of(s), 0
		g.falling = g.fy > seat_of(s) + 0.01
		R.holding = nil
	end

	-- R.O.B. is busy while any motor is still running (or a RESET sequence is still in progress)
	local function robot_busy()
		return #R.steps > 0
			or abs(R.ang - (R.stop_t - 2) * 60) > 0.5
			or abs(R.h - R.h_t) > 0.01
			or abs(R.claw - R.claw_t) > 0.01
	end

	on_command = function(cmd)
		-- The real R.O.B. firmware does not queue commands: anything that arrives while a motor is
		-- still running is ignored. Without this, a game that sends a command more than once
		-- (e.g. while a button is held) would make the robot move two steps instead of one.
		if robot_busy() and not cmd.force then
			return
		end
		local k = cmd.kind
		-- a move that would run past the end of the travel is not made (so the two-step moves that
		-- Gyromite uses only ever reach stops 5, 3 and 1: its high, middle and low positions)
		if k == "up" or k == "down" then
			local t = R.h_t + (k == "up" and cmd.n or -cmd.n)
			if t < 0 or t > 5 then return end
			R.h_t = t
		elseif k == "left" then R.stop_t = clamp(R.stop_t + 1, 0, 4)
		elseif k == "right" then R.stop_t = clamp(R.stop_t - 1, 0, 4)
		elseif k == "close" then R.claw_t = 0; R.claw_done = false
		elseif k == "open" then R.claw_t = 1; R.claw_done = false
		elseif k == "led" then R.led = true
		elseif k == "reset" then
			R.led = false
			R.claw_t = 1; R.claw_done = false
			-- seek rightmost / all the way up, then return to centre, still all the way up
			R.steps = { { stop = 0, h = 5 }, { stop = 2, h = 5 } }
		end
	end

	-- start the topple / fall / carry-back animation for a gyro standing on its station.
	-- dir = horizontal direction it falls toward, om0 = initial tipping speed (a knock gives it a push)
	local function start_topple(g, dir, om0)
		local a = (g.station - 2) * 60 * RAD
		local st = STATIONS[g.station]
		local tip = seat_of(g.station, g.loc) - GYRO_TIPL
		g.anim = { kind = "tip", x = HOLD_R * sin(a), y = tip, z = HOLD_R * cos(a), th = 0.03, om = om0, dir = dir, t = 0 }
		g.loc, g.falling = "anim", false
	end

	-- Gyromite: R.O.B.'s hands (and the claws on them) knock over any standing gyro whose spindle they
	-- run into, whether the body is turning, the arms are moving up / down, or the hands are closing.
	-- A gyro being gripped never collides: its spindle sits in the gap between the claws.
	local SPINDLE_V0, SPINDLE_V1, SPINDLE_R = GYRO_TIPL + 0.22, GYRO_TIPL + 0.22 + SPINDLE_LEN, 0.27   -- spindle, measured from the gyro tip
	local HAND_REACH = 3.4                                               -- claw + hand bar reach outward from the grip point
	local DISC_R = 1.08                                                  -- Stack-Up block radius (40 mm blocks)
	-- knock disk i (and every disk above it) off the stack at station st, toward direction dir
	local function knock_stack(st, i, dir)
		local stack = stacks[st]
		local a = (st - 2) * 60 * RAD
		local gx, gz = HOLD_R * sin(a), HOLD_R * cos(a)
		local y0 = STACK_SEAT + (i - 1) * DISC_H
		local ev = { s = st, discs = {}, t = 0, kind = "tumble" }
		for j = i, #stack do
			local k = j - i
			local sp = 0.09 + 0.03 * k                -- shoved sideways (upper disks a little harder)...
			local dd = dir + ((k % 2 == 1) and 0.25 or -0.15) * min(k, 2)   -- ...and fanning out
			ev.discs[#ev.discs + 1] = { color = stack[j].color, x = gx, z = gz,
				y = y0 + k * DISC_H + DISC_H / 2, vx = sin(dd) * sp, vz = cos(dd) * sp, vy = 0, on = true,
				th = 0, om = (0.07 + 0.02 * k) * DISC_TSCALE, dir = dd, rest = false }
		end
		for j = #stack, i, -1 do stack[j] = nil end
		knocked[#knocked + 1] = ev
	end
	-- bottom height of the disks R.O.B. is carrying: they hang in the grips, but rest on top of a
	-- stack (or empty tray) they are lowered onto instead of sinking into it
	local function held_disks_y(yc)
		local y = yc - DISC_H / 2
		local s = station_of_body()
		if s and stacks[s] then y = max(y, STACK_SEAT + #stacks[s] * DISC_H) end
		return y
	end
	-- disk height of the gyro R.O.B. is carrying (hangs from the top of its spindle, see the renderer)
	local function held_gyro_gb(yc)
		return max(yc + 0.27 - (0.62 + SPINDLE_LEN - 0.4), station_of_body() and seat_of(station_of_body()) or 0.4)
	end
	local function knock_check()
		if game ~= "gyromite" and game ~= "stackup" then return end
		local HAND_REACH = HAND_REACH                                    -- grip / claw + hand bar
		local yc = HAND_Y0 + R.h * ARM_STEP                              -- hand height (same as arm_y)
		local gap = claw_gap()
		local yaw = R.ang * RAD
		local c, sn = cos(yaw), sin(yaw)
		-- world positions of the hand centres, for the direction of the hit
		R.hand_prev = R.hand_prev or {}
		local hand_now = {}
		for side = -1, 1, 2 do
			local lx = side * (gap / 2 + HAND_REACH / 2)
			hand_now[side] = { lx * c + HOLD_R * sn, -lx * sn + HOLD_R * c }
		end
		-- does hand `side` (robot frame) touch a circle of radius rad at (lx, lz)?
		local function hand_hits(side, lx, lz, rad)
			local x0, x1 = gap / 2, gap / 2 + HAND_REACH
			local bx0, bx1 = side > 0 and x0 or -x1, side > 0 and x1 or -x0
			local hz = (game == "stackup") and 0.82 or 0.7                          -- (Stack-Up: the pad holder's depth)
			local qx, qz = clamp(lx, bx0, bx1), clamp(lz, HOLD_R - hz, HOLD_R + hz)    -- hand bar + claw
			return (lx - qx) ^ 2 + (lz - qz) ^ 2 < rad * rad
		end
		local function hit_dir(side, fallback)
			local p, q = R.hand_prev[side], hand_now[side]
			local vx, vz = p and q[1] - p[1] or 0, p and q[2] - p[2] or 0
			return (vx * vx + vz * vz > 1e-6) and math.atan(vx, vz) or fallback
		end
		-- Stack-Up: a hand or arm running into a stack knocks off the disk it hits and every disk above it.
		-- Hands and arms are solid as drawn (a hand block spans 0.45 above / below the hand height; the arm
		-- pieces are capsules along the shoulder piece and forearm), and a disk goes as soon as they touch it
		-- (a small margin). The disks R.O.B. is gripping (its own station, centred) are not knocked by the
		-- grippers' inner faces closing on them.
		-- the arms (both games): capsules in R.O.B.'s frame: x0, z0, x1, z1, r, y below / above yc, margin
		local arms = {}
		do
			local AX, AZ = ARM_APEX_X, ARM_APEX_Z
			local HX1 = AX + 3.4
			local d = arm_swing(gap / 2)
			local cd, sd = cos(d), sin(d)
			for side = -1, 1, 2 do
				local function T(x, z)                               -- as the renderer's arm transform
					local dx, dz = x - ARM_PIV_X, z - ARM_PIV_Z
					return side * (ARM_PIV_X + dx * cd + dz * sd), ARM_PIV_Z - dx * sd + dz * cd
				end
				local rx, rz = T(3.95, 0.0)                          -- shoulder root
				local ex, ez = T(5.5, 2.3)                           -- elbow
				local fx, fz = T(HX1 - 0.65, AZ - 0.1)               -- forearm end, over the hand
				arms[#arms + 1] = { rx, rz, ex, ez, 0.33, -0.6, 0.75, 0.05 }  -- shoulder piece (0.5 .. 0.7 wide)
				-- forearm: its centre line only. Open wide, the drawn forearm passes just outside the disks
				-- on the next tray (1.13 from its centre); it does not touch them.
				arms[#arms + 1] = { ex, ez, fx, fz, 0, -0.45, 0.5, 0 }
			end
		end
		-- does an arm piece touch an upright cylinder of radius rad at (lx, lz), from y0 to y1? (the side, or nil)
		local function arm_hits(lx, lz, rad, y0, y1, use_m)
			for _, q in ipairs(arms) do
				if yc + q[7] > y0 and yc + q[6] < y1 then
					local sx, sz = q[3] - q[1], q[4] - q[2]
					local t = clamp(((lx - q[1]) * sx + (lz - q[2]) * sz) / (sx * sx + sz * sz), 0, 1)
					local ddx, ddz = lx - (q[1] + sx * t), lz - (q[2] + sz * t)
					if ddx * ddx + ddz * ddz < (rad + q[5] + (use_m and q[8] or 0)) ^ 2 then return (q[1] > 0) and 1 or -1 end
				end
			end
			return nil
		end
		if game == "stackup" then
			local own = station_of_body()
			-- closed hands (done closing: a grab has already taken its disks) are solid on the stack under
			-- them too -- lowered further, or closed on nothing and lowered, they run into its disks
			local closed = R.claw_t == 0 and R.claw_done
			-- does a hand / arm touch a disk with its bottom at y0 (R.O.B. frame lx, lz)? m = the hands'
			-- touch margin, arm_m = use the arm pieces' own margins, grip = the disk is in the stack R.O.B.
			-- is centred over (closed hands are solid on it). Returns the side (+1 / -1) that touches, or nil.
			local function touch(y0, lx, lz, m, arm_m, grip)
				if yc + 0.45 > y0 and yc - 0.45 < y0 + DISC_H then
					if grip and closed then return 1 end
					local side = (hand_hits(1, lx, lz, DISC_R + m) and 1) or (hand_hits(-1, lx, lz, DISC_R + m) and -1)
					if side then return side end
				end
				return arm_hits(lx, lz, DISC_R, y0, y0 + DISC_H, arm_m)
			end
			-- a disk in the stack on station st (no touch margin for the stack R.O.B. is gripping)
			local function hit_at(st, y0, lx, lz)
				return touch(y0, lx, lz, (st == own) and 0 or 0.05, true, st == own)
			end
			R.stack_blocked = R.stack_blocked or {}
			for st = 0, 4 do
				local stack = stacks[st]
				if stack then
					local a = (st - 2) * 60 * RAD
					local gx, gz = HOLD_R * sin(a), HOLD_R * cos(a)
					local lx, lz = gx * c - gz * sn, gx * sn + gz * c
					for i = 1, #stack do
						local side = hit_at(st, STACK_SEAT + (i - 1) * DISC_H, lx, lz)
						if side then
							knock_stack(st, i, hit_dir(side, a))
							break
						end
					end
					-- which levels just above the stack has a hand / arm in them (where knocked disks would be
					-- put back)? -- level i = the i-th disk from the tray
					local blocked = {}
					for i = #stack + 1, #stack + 6 do
						if hit_at(st, STACK_SEAT + (i - 1) * DISC_H, lx, lz) then blocked[i] = true end
					end
					R.stack_blocked[st] = blocked
				end
			end
			-- loose (knocked-off) disks are solid too: a hand / arm running into one shoves it out of the
			-- way, and it slides / tumbles off from there (disks being carried back pass through)
			local function touches(lx, lz, y) return touch(y - DISC_H / 2, lx, lz, 0, false, false) end   -- y: the disk's centre
			for _, ev in ipairs(knocked) do
				if ev.kind == "tumble" then
					for _, d in ipairs(ev.discs) do
						local lx, lz = d.x * c - d.z * sn, d.x * sn + d.z * c
						local side = touches(lx, lz, d.y)
						if side then
							local dir = hit_dir(side, math.atan(d.x, d.z))
							local ux, uz = sin(dir), cos(dir)
							for _ = 1, 40 do                         -- out of the hand / arm, in the push direction
								d.x, d.z = d.x + ux * 0.05, d.z + uz * 0.05
								lx, lz = d.x * c - d.z * sn, d.x * sn + d.z * c
								if not touches(lx, lz, d.y) then break end
							end
							d.vx, d.vz = ux * 0.1, uz * 0.1
							d.rest, d.on, d.om = false, false, max(d.om or 0, 0.04)
							ev.t = 0                                     -- (it has to settle again before being put back)
						end
					end
				end
			end
			-- the stack R.O.B. is carrying knocks off any disks it runs into
			if R.holding then
				local hx, hz = HOLD_R * sn, HOLD_R * c                         -- carried stack centre (world)
				local hy0 = held_disks_y(yc)
				R.held_y0 = hy0                                              -- where the carried stack drops from
				local hy1 = hy0 + #R.holding * DISC_H
				for st = 0, 4 do
					local stack = stacks[st]
					if stack and #stack > 0 then
						local a = (st - 2) * 60 * RAD
						local gx, gz = HOLD_R * sin(a), HOLD_R * cos(a)
						local d2 = (gx - hx) ^ 2 + (gz - hz) ^ 2
						-- touching, but not stacked straight on top. The carried stack's bottom block has a cone
						-- underneath (0.38 deep) that sits in the recess on top of the block below: set down on a
						-- stack and then moved sideways, it catches in that recess and knocks the stack over.
						if d2 < (2 * DISC_R) ^ 2 and d2 > 0.15 * 0.15 then
							for i = 1, #stack do
								local y0 = STACK_SEAT + (i - 1) * DISC_H
								if hy1 > y0 + 0.02 and hy0 - 0.38 < y0 + DISC_H - 0.02 then
									knock_stack(st, i, hit_dir(1, math.atan(gx - hx, gz - hz)))
									break
								end
							end
						end
					end
				end
			end
			R.hand_prev = hand_now
			return
		end
		-- a gyro is two cylinders: disc (with its tip) and spindle, both measured from the disc bottom gb
		local function gyro_parts(gb)
			return { { 1.9, gb - GYRO_TIPL, gb + 0.22 }, { 0.27, gb + 0.22, gb + 0.22 + SPINDLE_LEN } }
		end
		local held = R.holding and game == "gyromite" and R.holding
		local held_parts, hx, hz
		if held then
			held_parts = gyro_parts(held_gyro_gb(yc))
			hx, hz = HOLD_R * sn, HOLD_R * c
			-- A carried gyro can only be swung over the spinner with the arms all the way up (it then just
			-- clears the tower). Lower down it runs into the tower (or the box): knocked out of the claws,
			-- it falls to the floor and is carried back to a free holder. (Lowered straight down onto the
			-- spinner from above, its leg goes into the tower: that is not a hit.)
			for s = 0, 4 do
				if STATIONS[s] and STATIONS[s].role == "spinner" then
					local a = (s - 2) * 60 * RAD
					local gx, gz = HOLD_R * sin(a), HOLD_R * cos(a)
					local gb = held_gyro_gb(yc)
					local tip = gb - GYRO_TIPL
					local tt = seat_of(s) - 0.12                          -- tower top
					local d = sqrt((hx - gx) ^ 2 + (hz - gz) ^ 2)
					local hit = d > 0.15 and ((d < 0.3 + 0.8 and tip < tt) or (d < 1.9 + 0.8 and gb < tt))
					for _, rr in ipairs({ HOLD_R - 2.3, HOLD_R - 0.8, HOLD_R + 0.6 }) do   -- the box (as in gyro_solids)
						local bx, bz = rr * sin(a), rr * cos(a)
						local db = sqrt((hx - bx) ^ 2 + (hz - bz) ^ 2)
						if (db < 0.3 + 1.6 and tip < 2.95) or (db < 1.9 + 1.6 and gb < 2.95) then hit = true end
					end
					if hit then
						R.holding = nil
						held.station = -1                                 -- (no station: it falls onto anything)
						held.anim = { kind = "fall", x = hx, y = tip, z = hz, th = 0.03, om = 0.08,
							dir = math.atan(hx - gx, hz - gz), vy = 0.001, vu = 0.06, t = 0 }
						held.loc, held.falling = "anim", false
						held = nil
						break
					end
				end
			end
		end
		for _, g in pairs(gyros) do
			if held and (g.loc == "dock" or g.loc == "tray" or g.loc == "spinner") and not g.falling then
				local a = (g.station - 2) * 60 * RAD
				local gx, gz = HOLD_R * sin(a), HOLD_R * cos(a)
				local st = STATIONS[g.station]
				local gb = seat_of(g.station, g.loc)
				local d = sqrt((gx - hx) ^ 2 + (gz - hz) ^ 2)
				local hit = false
				for _, p1 in ipairs(held_parts) do for _, p2 in ipairs(gyro_parts(gb)) do
					if d < p1[1] + p2[1] and p1[3] > p2[2] and p1[2] < p2[3] then hit = true end
				end end
				if hit then start_topple(g, hit_dir(1, math.atan(gx - hx, gz - hz)), 0.05) end
			end
			if (g.loc == "dock" or g.loc == "tray" or g.loc == "spinner") and not g.falling then
				local a = (g.station - 2) * 60 * RAD
				local gx, gz = HOLD_R * sin(a), HOLD_R * cos(a)
				local lx, lz = gx * c - gz * sn, gx * sn + gz * c               -- into the robot's frame
				local tip = seat_of(g.station, g.loc) - GYRO_TIPL
				-- The hands (0.35 above / below the hand height) and the arm pieces are solid against the
				-- gyro's leg, disc and spindle: whichever touches first knocks it over, toward where that
				-- hand / arm was moving (straight outward if it was not moving). A spindle sitting between
				-- the claws -- one they are closing on -- is not a hit.
				local between = R.gyro_in and g.station == station_of_body()
				local side
				for _, part in ipairs({ { 0.3, tip, tip + GYRO_TIPL }, { 1.9, tip + GYRO_TIPL, tip + SPINDLE_V0 },
						{ SPINDLE_R, tip + SPINDLE_V0, tip + SPINDLE_V1 } }) do
					local rad, y0, y1 = part[1], part[2], part[3]
					if not (between and rad == SPINDLE_R) and yc + 0.35 > y0 and yc - 0.35 < y1 then
						side = (hand_hits(1, lx, lz, rad) and 1) or (hand_hits(-1, lx, lz, rad) and -1)
					end
					side = side or arm_hits(lx, lz, rad, y0, y1, false)
					if side then break end
				end
				if side then start_topple(g, hit_dir(side, a), 0.05) end
			end
		end
		R.hand_prev = hand_now
	end

	-- One frame of a knocked-off Stack-Up disk. Trays, the stacks on them and other loose disks are
	-- solid: a disk slides across whatever it rests on, tips over the edge, falls (turning over) and
	-- lands on the floor, a tray, a stack or another disk. It is pushed out of anything it runs into
	-- from the side.
	local function disc_tumble_step(d)
		local pb = d.y - DISC_H / 2                    -- bottom before this frame
		if d.on then
			d.x, d.z = d.x + d.vx, d.z + d.vz           -- sliding on a surface
			d.vx, d.vz = d.vx * 0.985, d.vz * 0.985
			d.th = d.th * 0.8
		else
			d.vy = d.vy + DISC_GRAVITY
			d.x, d.y, d.z = d.x + d.vx, d.y - d.vy, d.z + d.vz
			d.th = d.th + d.om
		end
		-- solids: station columns (tray + stack) and the other loose disks
		local support, side_hit = 0, false
		for st = 0, 4 do
			local a = (st - 2) * 60 * RAD
			local cx, cz = HOLD_R * sin(a), HOLD_R * cos(a)
			local dx, dz = d.x - cx, d.z - cz
			local dh = sqrt(dx * dx + dz * dz)
			local n = #(stacks[st] or {})
			for _, part in ipairs({ { 1.5, STACK_SEAT - 0.47, STACK_SEAT }, { DISC_R, STACK_SEAT, STACK_SEAT + n * DISC_H } }) do
				local r, y0, y1 = part[1], part[2], part[3]
				if y1 > y0 and dh < r + DISC_R then
					if pb >= y1 - 0.05 then support = max(support, y1)          -- came from above: rests on it
					elseif d.y - DISC_H / 2 < y1 and d.y + DISC_H / 2 > y0 then  -- from the side: pushed out
						if dh < 1e-4 then dx, dz, dh = 1, 0, 1 end
						d.x, d.z = cx + dx / dh * (r + DISC_R), cz + dz / dh * (r + DISC_R)
						side_hit = true
					end
				end
			end
		end
		-- other loose disks: landed on from above it rests on them; run into from the side it is pushed
		-- out (they do not pass through each other while tumbling; disks being carried back do)
		for _, ev2 in ipairs(knocked) do
			if ev2.kind == "tumble" then
				for _, o in ipairs(ev2.discs) do
					if o ~= d then
						local dx, dz = d.x - o.x, d.z - o.z
						local dh = sqrt(dx * dx + dz * dz)
						local otop = o.y + DISC_H / 2
						if dh < 2 * DISC_R * 0.9 and pb >= otop - 0.05 then support = max(support, otop)
						elseif dh < 2 * DISC_R * 0.95 and abs(d.y - o.y) < DISC_H * 0.95 then
							if dh < 1e-4 then dx, dz, dh = 1, 0, 1 end
							local push = 2 * DISC_R * 0.95 - dh
							if o.rest then                                  -- a resting disk stays put
								d.x, d.z = d.x + dx / dh * push, d.z + dz / dh * push
							else                                            -- both moving: share the push
								d.x, d.z = d.x + dx / dh * push / 2, d.z + dz / dh * push / 2
								o.x, o.z = o.x - dx / dh * push / 2, o.z - dz / dh * push / 2
							end
							side_hit = true
						end
					end
				end
			end
		end
		if side_hit then d.vx, d.vz = -d.vx * 0.3, -d.vz * 0.3 end
		if support > 0 then
			if d.y - DISC_H / 2 <= support then
				d.y, d.vy = support + DISC_H / 2, 0
				d.on = true
				if d.vx * d.vx + d.vz * d.vz < 1e-5 then          -- stopped on top of something
					d.th, d.rest, d.on = 0, true, false
				end
			else
				d.on = false
			end
			return
		end
		d.on = false
		-- floor (allowing for the tilt while it lands)
		local low = d.y - (DISC_R * abs(sin(d.th)) + DISC_H / 2 * abs(cos(d.th)))
		if low <= 0 then
			d.y = d.y - low
			if d.vy > DISC_GRAVITY * 2 then
				d.vy, d.vx, d.vz, d.om = -d.vy * 0.25, d.vx * 0.5, d.vz * 0.5, d.om * 0.5
			else
				d.th = floor(d.th / math.pi + 0.5) * math.pi      -- settle flat
				d.y, d.rest = DISC_H / 2, true
			end
		end
	end

	-- Gyromite: solid parts a falling gyro can hit (vertical cylinders): the robot's base and each
	-- station's accessory (holder post + cup, button post + cap, spinner box + tower).
	local function gyro_solids(skip_station)
		local out = { { x = 0, z = 0, r = 4.6, y0 = 0, y1 = 2.74 }, { x = 0, z = 0, r = 2.6, y0 = 2.74, y1 = 2.92 } }  -- robot base, turntable
		for st = 0, 4 do
			local role = STATIONS[st] and STATIONS[st].role
			if role and st ~= skip_station then
				local a = (st - 2) * 60 * RAD
				local x, z = HOLD_R * sin(a), HOLD_R * cos(a)
				if role == "dock" then
					out[#out + 1] = { x = x, z = z, r = 0.5, y0 = 0, y1 = G_SEAT - 0.4 }
					out[#out + 1] = { x = x, z = z, r = 0.62, y0 = G_SEAT - 0.4, y1 = G_SEAT }
				elseif role == "tray" then
					local cap = seat_of(st, "tray") - GYRO_TIPL                     -- top of the cap
					out[#out + 1] = { x = x, z = z, r = 0.5, y0 = 0, y1 = cap - 0.3 }
					out[#out + 1] = { x = x, z = z, r = 0.88, y0 = cap - 0.3, y1 = cap }
				elseif role == "spinner" then
					-- the box (two circles along it) and the tower
					for _, rr in ipairs({ HOLD_R - 2.3, HOLD_R - 0.8, HOLD_R + 0.6 }) do
						out[#out + 1] = { x = rr * sin(a), z = rr * cos(a), r = 1.6, y0 = 0, y1 = 2.95 }
					end
					out[#out + 1] = { x = x, z = z, r = 0.8, y0 = 1.3, y1 = GYRO_SEAT + SPIN_UP - 0.12 }
				end
			end
		end
		return out
	end
	-- sample points on a gyro posed by animation A (tip at A.x/y/z, tilted A.th toward A.dir)
	local function gyro_points(A)
		local ct, st = cos(A.th), sin(A.th)
		local dx, dz, wx, wz = sin(A.dir), cos(A.dir), cos(A.dir), -sin(A.dir)
		local pts = {}
		local function add(u, v, w)
			local u2, v2 = u * ct + v * st, -u * st + v * ct
			pts[#pts + 1] = { A.x + u2 * dx + w * wx, A.y + v2, A.z + u2 * dz + w * wz }
		end
		add(0, 0, 0)
		for i = 0, 7 do local a = i * math.pi / 4 add(1.9 * cos(a), GYRO_TIPL + 0.11, 1.9 * sin(a)) end   -- disc rim
		for _, v in ipairs({ GYRO_TIPL + 0.22 + SPINDLE_LEN * 0.3, GYRO_TIPL + 0.22 + SPINDLE_LEN * 0.65, GYRO_TIPL + 0.22 + SPINDLE_LEN }) do add(0, v, 0) end                       -- spindle
		return pts
	end
	-- a falling gyro knocks over standing gyros it hits and is stopped by trays / holders / the base.
	-- Returns true when it came to rest on top of something this frame.
	local function gyro_fall_collide(g, A)
		local pts = gyro_points(A)
		-- standing gyros: domino
		for _, o in pairs(gyros) do
			if o ~= g and (o.loc == "dock" or o.loc == "tray" or o.loc == "spinner") and not o.falling then
				local a = (o.station - 2) * 60 * RAD
				local ox, oz = HOLD_R * sin(a), HOLD_R * cos(a)
				for _, p in ipairs(pts) do
					local dh = sqrt((p[1] - ox) ^ 2 + (p[3] - oz) ^ 2)
					local os_ = seat_of(o.station, o.loc)
					if (dh < 1.9 and p[2] > os_ - GYRO_TIPL and p[2] < os_ + 0.22)
						or (dh < 0.45 and p[2] >= os_ + 0.22 and p[2] < os_ + 0.22 + SPINDLE_LEN) then
						start_topple(o, math.atan(ox - A.x, oz - A.z), 0.06)
						break
					end
				end
			end
		end
		-- solid accessories and the robot's base
		local landed = false
		for _, c in ipairs(gyro_solids(g.station)) do
			for _, p in ipairs(pts) do
				local ex, ez = p[1] - c.x, p[3] - c.z
				local dh = sqrt(ex * ex + ez * ez)
				if dh < c.r and p[2] > c.y0 and p[2] < c.y1 then
					local up, side = c.y1 - p[2], c.r - dh
					if up <= side then                                  -- landed on top of it
						A.y = A.y + up
						landed = true
					else                                                -- ran into its side: pushed back out
						if dh < 1e-4 then ex, ez, dh = 1, 0, 1 end
						A.x, A.z = A.x + ex / dh * side, A.z + ez / dh * side
						A.vu = A.vu * 0.3
					end
					break
				end
			end
		end
		return landed
	end

	local function sim_step()
		-- motor speeds follow emulated time (one arm step in cfg.arm_step_time seconds), whatever rate
		-- this callback actually runs at
		local fps = cfg.frame_rate
		local okt, now = pcall(function() return manager.machine.time:as_double() end)
		if okt and type(now) == "number" then
			local dt = now - (R.sim_t or (now - 1 / fps))
			if dt < 0 or dt > 0.25 then dt = 1 / fps end                  -- state load / reset: one frame
			if dt < 0.002 then return end                                  -- called again within the same frame
			R.sim_t = now
			fps = 1 / dt
		end

		-- queued RESET steps start when the previous motion is done
		if #R.steps > 0 and abs(R.ang - (R.stop_t - 2) * 60) < 0.5 and abs(R.h - R.h_t) < 0.01
			and abs(R.claw - R.claw_t) < 0.01 then
			local st = table.remove(R.steps, 1)
			R.stop_t, R.h_t = st.stop, st.h
		end

		R.ang = approach(R.ang, (R.stop_t - 2) * 60, 60 / (cfg.body_step_time * fps))
		R.h = approach(R.h, R.h_t, 1 / (cfg.arm_step_time * fps))
		R.gyro_in = gyro_in_claws()
		R.claw = approach(R.claw, R.claw_t, 1 / (cfg.claw_time * fps))
		R.stop = floor((R.ang / 60) + 2.5)
		knock_check()

		-- whatever is held drops as soon as the claws have parted a little, not when they finish opening
		if R.holding and R.claw_t == 1 and R.claw > 0.12 then try_release() end
		if not R.claw_done and abs(R.claw - R.claw_t) < 0.001 then
			R.claw_done = true
			if R.claw_t == 0 then try_grab() else try_release() end
		end

		-- Stack-Up: released groups fall until they rest on top of the stack below
		for i = #falling, 1, -1 do
			local f = falling[i]
			f.vy = f.vy + DISC_GRAVITY
			f.y = f.y - f.vy
			local st = stacks[f.s]
			if f.y <= STACK_SEAT + #st * DISC_H then
				for _, d in ipairs(f.group) do st[#st + 1] = d end
				table.remove(falling, i)
			end
		end

		-- Stack-Up: knocked disks tumble off, land flat on the floor, rest a moment, then are carried
		-- back (in their original order) and dropped onto the tray they came from
		-- (would they land where a hand / arm is? -- only the levels they will fill on that stack)
		local function restore_blocked(ev)
			local b = R.stack_blocked and R.stack_blocked[ev.s]
			if not b then return false end
			local n0 = #(stacks[ev.s] or {})
			for i = n0 + 1, n0 + #ev.discs do if b[i] then return true end end
			return false
		end
		for ei = #knocked, 1, -1 do
			local ev = knocked[ei]
			if ev.kind == "tumble" then
				local all = true
				for _, d in ipairs(ev.discs) do
					if not d.rest then
						all = false
						disc_tumble_step(d)
					end
				end
				-- (they wait on the floor while R.O.B.'s hands / arms are in the way above their tray, so
				-- they are not put straight back into them and knocked off again)
				if all and not restore_blocked(ev) then ev.t = ev.t + 1 end
				if all and ev.t > 50 then
					ev.kind, ev.t = "carry", 0
					local a = (ev.s - 2) * 60 * RAD
					local base = STACK_SEAT + #(stacks[ev.s] or {}) * DISC_H + 0.8
					for k, d in ipairs(ev.discs) do
						d.sx, d.sy, d.sz, d.sth = d.x, d.y, d.z, d.th
						d.tx, d.ty, d.tz = HOLD_R * sin(a), base + (k - 1) * DISC_H + DISC_H / 2, HOLD_R * cos(a)
					end
				end
			elseif ev.kind == "carry" then
				local N = 100
				-- hold just short of the drop while the hands / arms are in the way
				if ev.t < N - 1 or not restore_blocked(ev) then ev.t = ev.t + 1 end
				local e = ev.t / N
				local ee = e * e * (3 - 2 * e)
				for _, d in ipairs(ev.discs) do
					d.x = d.sx + (d.tx - d.sx) * ee
					d.z = d.sz + (d.tz - d.sz) * ee
					d.y = d.sy + (d.ty - d.sy) * ee + 2.5 * sin(math.pi * e)
					d.th = d.sth * (1 - ee)
				end
				if ev.t >= N then
					local group = {}
					for _, d in ipairs(ev.discs) do group[#group + 1] = { color = d.color } end
					falling[#falling + 1] = { group = group, s = ev.s, y = ev.discs[1].y - DISC_H / 2, vy = 0 }
					table.remove(knocked, ei)
				end
			end
		end

		-- things lying on the floor (toppled gyros, knocked-off disks) cannot overlap: push them apart
		do
			local items = {}
			for _, g in pairs(gyros) do
				local A = g.anim
				if A and (A.kind == "rest" or (A.kind == "fall" and A.vy == 0)) then
					items[#items + 1] = { obj = A, r = 1.7, ox = sin(A.dir) * 1.2, oz = cos(A.dir) * 1.2 }  -- footprint ~ disc centre
				end
			end
			for _, ev in ipairs(knocked) do
				if ev.kind == "tumble" then
					for _, d in ipairs(ev.discs) do if d.rest and d.y < DISC_H then items[#items + 1] = { obj = d, r = DISC_R, ox = 0, oz = 0 } end end
				end
			end
			for i = 1, #items do for j = i + 1, #items do
				local p, q = items[i], items[j]
				local dx = (q.obj.x + q.ox) - (p.obj.x + p.ox)
				local dz = (q.obj.z + q.oz) - (p.obj.z + p.oz)
				local d = sqrt(dx * dx + dz * dz)
				local need = p.r + q.r
				if d < need then
					if d < 1e-4 then dx, dz, d = 1, 0, 1 end
					local push = min(0.08, (need - d) / 2)                     -- slide apart over a few frames
					p.obj.x, p.obj.z = p.obj.x - dx / d * push, p.obj.z - dz / d * push
					q.obj.x, q.obj.z = q.obj.x + dx / d * push, q.obj.z + dz / d * push
				end
			end end
		end

		-- gyro spin model
		local spin_up = 1 / (cfg.spinup_time * fps)
		local spin_down = 1 / (cfg.spin_duration * fps)
		tray_down.red, tray_down.blue = 0, 0                 -- how far down each button is being pushed (0..1)
		for _, g in pairs(gyros) do
			-- A gyro set free on a red / blue button only balances there while it spins. A slow one topples
			-- off, falls to the floor, lies there a moment, and is then carried back to a free holder.
			if g.loc == "tray" and not g.falling and g.spin < (cfg.balance_spin or 0.15) then
				start_topple(g, (g.station - 2) * 60 * RAD, 0)
			end
			if g.loc == "anim" then
				local A = g.anim
				local dx, dz = sin(A.dir), cos(A.dir)
				local TH_REST = math.pi - math.atan((0.22 + SPINDLE_LEN) / 1.9)   -- lying on its disc edge and spindle end
				if A.kind == "tip" then
					-- pivots on its tip on the button, accelerating like a falling stick
					A.om = A.om + GRAVITY / 0.9 * sin(A.th)
					A.th = A.th + A.om
					if A.th > 0.75 then A.kind, A.vy, A.vu, A.om = "fall", 0, clamp(A.om * 0.7, 0.05, 0.09), min(A.om, 0.11) end
				elseif A.kind == "fall" then
					-- off the button: falls to the floor while still turning over, then rolls onto its side
					A.vy = A.vy + GRAVITY
					A.y = A.y - A.vy
					A.x, A.z = A.x + dx * A.vu, A.z + dz * A.vu
					if A.th < TH_REST then A.th = min(TH_REST, A.th + min(A.om, 0.11)) end
					local on_top = gyro_fall_collide(g, A)
					local low = A.y + gyro_lowest(A.th)
					if low <= 0 or on_top then
						if low <= 0 then A.y = A.y - low end
						A.vu = A.vu * 0.8
						if A.vy > GRAVITY * 2 then A.vy = -A.vy * 0.25 else A.vy = 0 end
						A.om = max(A.om * 0.92, 0.04)            -- rolls over onto its side
						if (A.th >= TH_REST or on_top) and A.vy == 0 then A.kind, A.t = "rest", 0 end
					end
				elseif A.kind == "rest" then
					A.t = A.t + 1
					if A.t > 50 then
						-- find the first free gyro holder and carry the gyro over to it (one another gyro is
						-- already being carried to counts as taken)
						local target
						for s2 = 0, 4 do
							local st2 = STATIONS[s2]
							if st2 and st2.role == "dock" then
								local used = false
								for _, o in pairs(gyros) do
									if o ~= g then
										if o.loc == "anim" then
											if o.anim and o.anim.target == s2 then used = true end
										elseif o.loc ~= "held" and o.station == s2 then
											used = true
										end
									end
								end
								if not used then target = s2 break end
							end
						end
						if target then
							A.kind, A.t, A.target = "carry", 0, target
							A.a0, A.r0, A.y0, A.th0 = math.atan(A.x, A.z), sqrt(A.x * A.x + A.z * A.z), A.y, A.th
						end
					end
				elseif A.kind == "carry" then
					local N = 100
					A.t = A.t + 1
					local e = A.t / N
					local ee = e * e * (3 - 2 * e)           -- ease in / out
					local a1 = (A.target - 2) * 60 * RAD
					local da = a1 - A.a0
					if da > math.pi then da = da - 2 * math.pi elseif da < -math.pi then da = da + 2 * math.pi end
					local ang = A.a0 + da * ee
					local r = A.r0 + (HOLD_R - A.r0) * ee + 2.0 * sin(math.pi * e)   -- swing wide round the robot
					local y1 = G_SEAT + 0.3                                               -- tip just above the holder
					A.x, A.z = r * sin(ang), r * cos(ang)
					A.y = A.y0 + (y1 - A.y0) * ee + 3.0 * sin(math.pi * e)
					A.th = A.th0 * (1 - min(1, e * 1.6))                             -- turned upright early
					if A.t >= N then
						-- set down: it drops the last bit onto the holder
						g.loc, g.station, g.anim = "dock", A.target, nil
						g.fy, g.fvy, g.falling = y1 + GYRO_TIPL, 0, true
					end
				end
			end
			-- a released gyro falls under gravity onto its station and bounces a little before settling
			if g.falling then
				local st = STATIONS[g.station]
				local rest = seat_of(g.station, g.loc)
				g.fvy = g.fvy + GRAVITY
				g.fy = g.fy - g.fvy
				if g.fy <= rest then
					g.fy = rest
					if g.fvy > GRAVITY * 2 then g.fvy = -g.fvy * BOUNCE else g.falling, g.fvy = false, 0 end
				end
			end
			-- on the spinner's drive: set free there, or still held in the claws once lowered all the way
			-- onto it (stop 3) -- it does not spin while it is still being lowered into the tower
			local driven = g.loc == "spinner" and not g.falling
			if g.loc == "held" then
				local s = station_of_body()
				driven = s ~= nil and STATIONS[s] and STATIONS[s].role == "spinner"
					and HAND_Y0 + R.h * ARM_STEP + 0.27 - (0.22 + SPINDLE_LEN) <= seat_of(s) + 0.01
			end
			if driven then g.spin = min(1, g.spin + spin_up)
			elseif g.loc == "dock" and not g.falling then
				g.spin = max(0, g.spin - 1 / (cfg.dock_stop_time * fps))      -- the holder brakes it
			else g.spin = max(0, g.spin - spin_down) end
			g.phase = (g.phase + g.spin * 0.7) % (2 * math.pi)   -- ~6.7 turns/s at full spin (the rim band blurs to grey)
			-- a (spinning) gyro standing on a red / blue button pushes it all the way down by its weight
			if g.loc == "tray" and not g.falling then
				local st = STATIONS[g.station]
				if st and st.color then tray_down[st.color] = 1 end
			end
			-- a gyro held in the claws, spinning or not, lowered onto a button: hanging from the top of its
			-- spindle it sinks the button as far as the hands let it down (all the way, one stop lower)
			if g.loc == "held" then
				local s = station_of_body()
				local st = s and STATIONS[s]
				if st and st.role == "tray" and st.color then
					local hang = HAND_Y0 + R.h * ARM_STEP + 0.27 - (0.22 + SPINDLE_LEN)   -- disc bottom, hanging
					local d = clamp((GYRO_SEAT + BTN_LIFT + TRAY_TRAVEL - hang) / TRAY_TRAVEL, 0, 1)
					tray_down[st.color] = max(tray_down[st.color], d)
				end
			end
		end
		-- the buttons sink under a gyro's weight (and spring back up once it is lifted off)
		for _, c in ipairs({ "red", "blue" }) do
			R.press[c] = approach(R.press[c] or 0, tray_down[c], ARM_STEP / TRAY_TRAVEL / ((cfg.tray_step_time or 1.5) * fps))
		end
	end

	---------------------------------------------------------------------------
	-- 4. PLAYER 2 PORT INTERFACE
	---------------------------------------------------------------------------
	local p2_fields = {}           -- name -> field
	local btn_field = { red = nil, blue = nil }
	local btn_state = { red = false, blue = false }

	local function discover_ports()
		p2_fields = {}
		local ok, ports = pcall(function() return manager.machine.ioport.ports end)
		if not ok or not ports then return end
		local names = {}
		for tag, port in pairs(ports) do
			for fname, field in pairs(port.fields) do
				local is_p2 = false
				local okp, player = pcall(function() return field.player end)
				if okp and player == 1 then is_p2 = true end
				if tag:lower():find("ctrl2", 1, true) or fname:upper():find("^P2") then is_p2 = true end
				if is_p2 then
					p2_fields[#p2_fields + 1] = { tag = tag, name = fname, field = field }
					names[#names + 1] = fname
				end
			end
		end
		if #names == 0 then return end              -- ports not populated yet: tried again on a later frame

		-- Gyromite: the blue button presses controller 2 A, the red button presses controller 2 B.
		-- MAME names these fields "P2 A" / "P2 B" ("P2 Button A" / "P2 Button B" in some versions).
		local function pick(letter)
			for _, f in ipairs(p2_fields) do
				local n = f.name:lower()
				if n == "p2 " .. letter or n == "p2 button " .. letter then return f end
			end
			return nil
		end
		btn_field.blue = pick("a")
		btn_field.red = pick("b")
	end

	local function drive_button(which)
		local want = (R.press[which] or 0) >= 1         -- the switch closes at the bottom of the button's travel
		if want == btn_state[which] then return end
		btn_state[which] = want
		local f = btn_field[which]
		if not f then return end
		pcall(function()
			if want then f.field:set_value(1)
			elseif f.field.clear_value then f.field:clear_value()
			else f.field:set_value(0) end
		end)
	end

	---------------------------------------------------------------------------
	-- 5. TINY SOFTWARE 3D RENDERER
	---------------------------------------------------------------------------
	local CAM_YAW0, CAM_PITCH0 = (180 + 24) * RAD, 18 * RAD   -- camera starts in front of R.O.B.
	local cam = { yaw = CAM_YAW0, pitch = CAM_PITCH0, dist = 47, focal = 194, ty = 5.6 }
	local light_world = { -0.45, 0.80, 0.55 }
	local light_view
	local faces
	local face_seq = 0      -- running number of the faces added (their order for faces at the same depth)
	local front_view = true
	local faces_sorted = false  -- faces are already in painting order (build_scene merges them)
	local face_bias = 0      -- subtract from a face's sort depth so decals draw in front of their surface

	local function rot_y(x, z, a)
		local c, s = cos(a), sin(a)
		return x * c + z * s, -x * s + z * c
	end

	-- push one convex face. pts are world-space {x,y,z}; solid centre is used to orient normals.
	local FACE_MAX = 0.9                -- largest face edge before it is cut up for sorting (scene units)
	local split_faces = false           -- only the shoulder housing / arms need it (it costs render time)
	local face_xf = nil                 -- optional transform applied to a section's faces (see the head / housing)
	local decal_any_view = false        -- decals that face away from the robot: keep their bias from any angle
	-- painting order: back to front; faces at the same depth keep the order they were added in
	-- (table.sort is not stable: an unstable order made overlapping pieces swap from frame to frame)
	local function face_order(a, b)
		local da, db = a.depth, b.depth
		if da ~= db then return da > db end
		return a.seq < b.seq
	end
	-- camera rotation, worked out once per frame (cam_prepare) instead of for every vertex
	local cam_prepare, w2v, add_face
	do
	local W_CY, W_SY, W_CP, W_SP, W_FLIP, W_TY, W_DIST = 1, 0, 1, 0, true, 0, 0
	-- (the same, as add_face uses them: mirror factor, camera target x / z, camera and light directions)
	local W_FL, W_TX, W_TZ, W_WX, W_WY, W_WZ, W_LX, W_LY, W_LZ = -1, 0, 0, 0, 0, 0, 0, 1, 0
	local XF_FN, XF_MEMO = nil, {}      -- add_face: the last face_xf, and the points it has transformed
	function cam_prepare()
		W_CY, W_SY = cos(cam.yaw), sin(cam.yaw)
		W_CP, W_SP = cos(cam.pitch), sin(cam.pitch)
		W_FLIP = not cfg.mirror_lr                 -- +X is R.O.B.'s left, which is screen-right in a front view
		W_TY, W_DIST = cam.ty, cam.dist
		-- the camera's position in world space and the unit light direction, so faces can be culled and
		-- lit before their vertices are transformed
		local z1 = -W_CP * W_DIST
		cam._wx, cam._wy, cam._wz = -W_SY * z1 + (cam.tx or 0), W_TY + W_SP * W_DIST, W_CY * z1 + (cam.tz or 0)
		local lx, ly, lz = light_world[1], light_world[2], light_world[3]
		local ll = sqrt(lx * lx + ly * ly + lz * lz)
		cam._lx, cam._ly, cam._lz = lx / ll, ly / ll, lz / ll
		cam._xf = { W_CY, W_SY, W_CP, W_SP, W_FLIP and -1 or 1, cam.ty, W_DIST, cam.tx or 0, cam.tz or 0 }
		W_FL, W_TX, W_TZ = W_FLIP and -1 or 1, cam.tx or 0, cam.tz or 0
		W_WX, W_WY, W_WZ = cam._wx, cam._wy, cam._wz
		W_LX, W_LY, W_LZ = cam._lx, cam._ly, cam._lz
	end
	function w2v(x, y, z)
		y = y - cam.ty
		local x1 = W_CY * x + W_SY * z
		local z1 = -W_SY * x + W_CY * z
		if W_FLIP then x1 = -x1 end
		return x1, W_CP * y + W_SP * z1, -W_SP * y + W_CP * z1 + W_DIST
	end

	-- (helpers for add_face, defined once rather than per face)
	local function put_face(p, bias, r, g, b)
		local dz = 0
		for i = 1, #p do dz = dz + p[i][3] end
		face_seq = face_seq + 1
		faces[#faces + 1] = { vs = p, depth = dz / #p - bias, r = r, g = g, b = b, seq = face_seq }
	end
	local function lerp3(a, c, t) return { a[1] + (c[1] - a[1]) * t, a[2] + (c[2] - a[2]) * t, a[3] + (c[3] - a[3]) * t } end
	local function dist3(a, c)
		local dx, dy, dz = a[1] - c[1], a[2] - c[2], a[3] - c[3]
		return sqrt(dx * dx + dy * dy + dz * dz)
	end
	local function tri_split(a, b2, c, depth, bias, r, g, b)
		local ab, bc, ca = dist3(a, b2), dist3(b2, c), dist3(c, a)
		if depth > 6 or max(ab, bc, ca) <= FACE_MAX then put_face({ a, b2, c }, bias, r, g, b) return end
		local m1, m2, m3 = lerp3(a, b2, 0.5), lerp3(b2, c, 0.5), lerp3(c, a, 0.5)
		tri_split(a, m1, m3, depth + 1, bias, r, g, b) tri_split(m1, b2, m2, depth + 1, bias, r, g, b)
		tri_split(m3, m2, c, depth + 1, bias, r, g, b) tri_split(m1, m2, m3, depth + 1, bias, r, g, b)
	end
	function add_face(pts, cx, cy, cz, rgb, tex)
		if face_xf then                                    -- a section's correction transform (world space)
			-- (points shared by several faces are transformed once: remembered per transform)
			if face_xf ~= XF_FN then XF_FN, XF_MEMO = face_xf, {} end
			local q, M = {}, XF_MEMO
			for i = 1, #pts do
				local p = pts[i]
				local t = M[p]
				if not t then t = face_xf(p) M[p] = t end
				q[i] = t
			end
			pts = q
			local c = face_xf({ cx, cy, cz })
			cx, cy, cz = c[1], c[2], c[3]
		end
		-- orient, cull and light in world space (the camera transform only rotates / mirrors / moves,
		-- so the answers are the same), and transform the vertices only of faces that are kept
		local n = #pts
		local p1, p2, p3 = pts[1], pts[2], pts[3]
		local ax, ay, az = p2[1] - p1[1], p2[2] - p1[2], p2[3] - p1[3]
		local bx, by, bz = p3[1] - p1[1], p3[2] - p1[2], p3[3] - p1[3]
		local nx, ny, nz = ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
		-- (which way the face points and whether it faces the camera: any point of a flat face gives
		-- the same answers, so its first point is used, and the normal's length does not matter yet)
		local px, py, pz = p1[1], p1[2], p1[3]
		local o = nx * (px - cx) + ny * (py - cy) + nz * (pz - cz)
		if o < 0 then nx, ny, nz = -nx, -ny, -nz end
		if nx * (px - W_WX) + ny * (py - W_WY) + nz * (pz - W_WZ) >= 0 then return end   -- back face
		local len = sqrt(nx * nx + ny * ny + nz * nz)
		if len < 1e-9 then return end
		nx, ny, nz = nx / len, ny / len, nz / len

		-- (the camera transform, inlined: same as w2v)
		local CY, SY, CP, SP, FL, TY, DI, TX, TZ = W_CY, W_SY, W_CP, W_SP, W_FL, W_TY, W_DIST, W_TX, W_TZ
		local vs = {}
		local fx, fy, fz = 0, 0, 0
		for i = 1, n do
			local q = pts[i]
			local x, y, z = q[1] - TX, q[2] - TY, q[3] - TZ
			local x1 = (CY * x + SY * z) * FL
			local z1 = -SY * x + CY * z
			local vy, vz = CP * y + SP * z1, -SP * y + CP * z1 + DI
			vs[i] = { x1, vy, vz }
			fx, fy, fz = fx + x1, fy + vy, fz + vz
		end
		fx, fy, fz = fx / n, fy / n, fz / n

		local lam = max(0, nx * W_LX + ny * W_LY + nz * W_LZ)
		local shade = 0.42 + 0.68 * lam
		if tex then rgb = tex.avg end
		local r, g, b = rgb[1] * shade, rgb[2] * shade, rgb[3] * shade
		-- decals (face_bias > 0) are pulled forward so they win against the face they lie on. Seen from
		-- the front only (from behind, a pulled-forward decal would show through, e.g. the eyes through
		-- the back of the head); in the shoulder area, where big faces are cut into pieces, the pull is
		-- larger to cover the depth spread of the pieces.
		local bias = split_faces and face_bias * 2.5 or ((front_view or decal_any_view) and face_bias or 0)
		-- The faces are drawn back to front by their centre's depth. A big face next to small ones
		-- sorts badly (its centre can be far from the part that overlaps them), so faces bigger than
		-- FACE_MAX are cut into pieces first.
		if tex then
			local dz = 0
			for i = 1, n do dz = dz + vs[i][3] end
			face_seq = face_seq + 1
			faces[#faces + 1] = { vs = vs, depth = dz / n - bias, r = r, g = g, b = b, tex = tex, shade = shade, seq = face_seq }
			return
		end
		if not split_faces then
			face_seq = face_seq + 1
			faces[#faces + 1] = { vs = vs, depth = fz - bias, r = r, g = g, b = b, seq = face_seq }
			return
		end
		local function put(p) put_face(p, bias, r, g, b) end
		local dist = dist3
		if n == 4 then
			local nu = min(12, ceil(max(dist(vs[1], vs[2]), dist(vs[4], vs[3])) / FACE_MAX))
			local nv = min(12, ceil(max(dist(vs[2], vs[3]), dist(vs[1], vs[4])) / FACE_MAX))
			if nu * nv == 1 then put(vs) return end
			-- grid[i][j]: lerp(lerp(v1, v2, i / nu), lerp(v4, v3, i / nu), j / nv), worked out in place
			local v1, v2, v3, v4 = vs[1], vs[2], vs[3], vs[4]
			local grid = {}
			for i = 0, nu do
				local t = i / nu
				local ax, ay, az = v1[1] + (v2[1] - v1[1]) * t, v1[2] + (v2[2] - v1[2]) * t, v1[3] + (v2[3] - v1[3]) * t
				local bx, by, bz = v4[1] + (v3[1] - v4[1]) * t, v4[2] + (v3[2] - v4[2]) * t, v4[3] + (v3[3] - v4[3]) * t
				local dx, dy, dz = bx - ax, by - ay, bz - az
				local row = {}
				for j = 0, nv do
					local u = j / nv
					row[j] = { ax + dx * u, ay + dy * u, az + dz * u }
				end
				grid[i] = row
			end
			for i = 0, nu - 1 do
				for j = 0, nv - 1 do put({ grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1] }) end
			end
		else
			local big = false
			for i = 1, n do if dist(vs[i], vs[i % n + 1]) > FACE_MAX then big = true break end end
			if not big and n == 3 then put(vs) return end
			if not big then
				-- small polygon: still split if it is wide across (a long thin fan)
				local wide = false
				for i = 1, n do for j = i + 2, n do if dist(vs[i], vs[j]) > FACE_MAX * 1.5 then wide = true end end end
				if not wide then put(vs) return end
			end
			local c = { fx, fy, fz }
			for i = 1, n do tri_split(c, vs[i], vs[i % n + 1], 0, bias, r, g, b) end
		end
	end
	end

	local BOX_FACES = {
		{ 1, 2, 4, 3 }, { 5, 6, 8, 7 }, { 1, 2, 6, 5 },
		{ 3, 4, 8, 7 }, { 1, 3, 7, 5 }, { 2, 4, 8, 6 },
	}

	-- The two building blocks every solid is made of. (mx, my, mz) is a point inside the solid: faces
	-- are turned to face away from it.
	-- a six-sided solid from its 8 corners (corner i: bit 2 / 1 / 0 = +x / +y / +z side, see BOX_FACES)
	local function emit_hexa(c, mx, my, mz, rgb)
		for _, f in ipairs(BOX_FACES) do
			add_face({ c[f[1]], c[f[2]], c[f[3]], c[f[4]] }, mx, my, mz, rgb)
		end
	end
	-- the side quads between two rings of points with the same count (face i joins point i to i + 1),
	-- then up to two caps: cap1 in colour col1, cap2 in col2
	local function emit_rings(a, b, mx, my, mz, rgb, cap1, col1, cap2, col2)
		local n = #a
		for i = 1, n do
			local j = i % n + 1
			add_face({ a[i], a[j], b[j], b[i] }, mx, my, mz, rgb)
		end
		if cap1 then add_face(cap1, mx, my, mz, col1) end
		if cap2 then add_face(cap2, mx, my, mz, col2) end
	end

	-- box centred at cx,cy,cz with half extents; yaw = rotation about the world Y axis (the robot's axis)
	local function emit_box(cx, cy, cz, hx, hy, hz, rgb, yaw)
		yaw = yaw or 0
		local c = {}
		for i = 0, 7 do
			local sx = (i & 4) ~= 0 and 1 or -1
			local sy = (i & 2) ~= 0 and 1 or -1
			local sz = (i & 1) ~= 0 and 1 or -1
			local x, z = rot_y(cx + sx * hx, cz + sz * hz, yaw)
			c[i + 1] = { x, cy + sy * hy, z }
		end
		local mx, mz = rot_y(cx, cz, yaw)
		emit_hexa(c, mx, cy, mz, rgb)
	end

	-- n-sided vertical prism / cylinder
	local function emit_prism(cx, cy, cz, rad, hgt, sides, rgb, yaw, spin, top_rgb)
		yaw, spin = yaw or 0, spin or 0
		local bot, top = {}, {}
		for i = 0, sides - 1 do
			local a = spin + i * 2 * math.pi / sides
			local x, z = rot_y(cx + rad * cos(a), cz + rad * sin(a), yaw)
			bot[i + 1] = { x, cy - hgt / 2, z }
			top[i + 1] = { x, cy + hgt / 2, z }
		end
		local mx, mz = rot_y(cx, cz, yaw)
		emit_rings(bot, top, mx, cy, mz, rgb, top, top_rgb or rgb, bot, rgb)
	end

	-- cylinder lying along the (yawed) Z axis: arms, eye lenses
	local function emit_zprism(cx, cy, cz, rad, len, sides, rgb, yaw, cap_rgb)
		yaw = yaw or 0
		local a0, a1 = {}, {}
		for i = 0, sides - 1 do
			local a = i * 2 * math.pi / sides
			local lx, ly = cx + rad * cos(a), cy + rad * sin(a)
			local x0, z0 = rot_y(lx, cz - len / 2, yaw)
			local x1, z1 = rot_y(lx, cz + len / 2, yaw)
			a0[i + 1] = { x0, ly, z0 }
			a1[i + 1] = { x1, ly, z1 }
		end
		local mx, mz = rot_y(cx, cz, yaw)
		emit_rings(a0, a1, mx, cy, mz, rgb, a1, cap_rgb or rgb, a0, rgb)
	end


	-- rounded-rectangle outline (CCW), used as an extrusion profile
	local function rrect(hw, hh, r, seg)
		local pts = {}
		local corners = {
			{ hw - r, hh - r, 0 }, { -(hw - r), hh - r, 90 },
			{ -(hw - r), -(hh - r), 180 }, { hw - r, -(hh - r), 270 },
		}
		for _, c in ipairs(corners) do
			for i = 0, seg do
				local a = (c[3] + i * 90 / seg) * RAD
				pts[#pts + 1] = { c[1] + r * cos(a), c[2] + r * sin(a) }
			end
		end
		return pts
	end


	-- tapered vertical cylinder: radius rb at y0, rt at y0 + h
	local function emit_frustum(cx, y0, cz, rb, rt, h, sides, rgb, yaw, spin, top_rgb)
		yaw, spin = yaw or 0, spin or 0
		local bot, top = {}, {}
		for i = 0, sides - 1 do
			local a = spin + i * 2 * math.pi / sides
			local bx, bz = rot_y(cx + rb * cos(a), cz + rb * sin(a), yaw)
			local tx, tz = rot_y(cx + rt * cos(a), cz + rt * sin(a), yaw)
			bot[i + 1] = { bx, y0, bz }
			top[i + 1] = { tx, y0 + h, tz }
		end
		local mx, mz = rot_y(cx, cz, yaw)
		emit_rings(bot, top, mx, y0 + h / 2, mz, rgb, top, top_rgb or rgb, bot, rgb)
	end

	-- Faces are rasterised (painter's order) into a small in-memory framebuffer, then the buffer is
	-- sent to MAME as merged rectangles. This keeps the number of draw calls low (the earlier
	-- "one box per scanline per face" approach sent thousands of calls per frame).
	local fb, fb_w, fb_h = {}, 0, 0
	local last_rects = 0

	local function fb_clear(w, h)
		if w ~= fb_w or h ~= fb_h then fb, fb_w, fb_h = {}, w, h end
		for i = 1, w * h do fb[i] = 0 end
	end

	-- Fill a convex polygon (screen points xs[i], ys[i], i = 1..n) on a w x h pixel grid: a pixel is
	-- inside when its centre is. Covered runs go to box(target, first column, row, width, rows, col),
	-- one call for consecutive rows with the same run (rows and columns from 0). Both drawing paths (the
	-- ARGB bitmap and the rectangle framebuffer) use it.
	-- From the top point the outline is walked down both ways at once (one edge at a time on each
	-- side, an edge covering y0 <= y < y1), the two crossings stepped along from row to row.
	local function fill_poly(xs, ys, n, w, h, box, target, col)
		local top, miny, maxy = 1, ys[1], ys[1]
		for i = 2, n do
			local y = ys[i]
			if y < miny then miny, top = y, i elseif y > maxy then maxy = y end
		end
		-- the rows whose centre miny <= r + 0.5 < maxy
		local r0, r1 = ceil(miny - 0.5), ceil(maxy - 0.5) - 1
		if r0 < 0 then r0 = 0 end
		if r1 > h - 1 then r1 = h - 1 end
		if r1 < r0 then return end
		local yy = r0 + 0.5
		-- side A walks forward (i, i + 1, ...), side B backward; aj / bj: the end point of the current edge
		local ai, aj, bi, bj = top, top % n + 1, top, (top - 2) % n + 1
		local ay1, by1 = ys[aj], ys[bj]
		while ay1 <= yy do ai, aj = aj, aj % n + 1 ay1 = ys[aj] end
		while by1 <= yy do bi, bj = bj, (bj - 2) % n + 1 by1 = ys[bj] end
		local ys0 = ys[ai]
		local sa = (xs[aj] - xs[ai]) / (ay1 - ys0)
		local xa = xs[ai] + (yy - ys0) * sa - 0.5                -- (- 0.5: column centres)
		ys0 = ys[bi]
		local sb = (xs[bj] - xs[bi]) / (by1 - ys0)
		local xb = xs[bi] + (yy - ys0) * sb - 0.5
		local w1 = w - 1
		local s0, sc0, sc1 = nil, 0, -1
		for r = r0, r1 do
			if yy >= ay1 then
				repeat ai, aj = aj, aj % n + 1 ay1 = ys[aj] until ay1 > yy
				ys0 = ys[ai]
				sa = (xs[aj] - xs[ai]) / (ay1 - ys0)
				xa = xs[ai] + (yy - ys0) * sa - 0.5
			end
			if yy >= by1 then
				repeat bi, bj = bj, (bj - 2) % n + 1 by1 = ys[bj] until by1 > yy
				ys0 = ys[bi]
				sb = (xs[bj] - xs[bi]) / (by1 - ys0)
				xb = xs[bi] + (yy - ys0) * sb - 0.5
			end
			-- columns ceil(left - 0.5) .. floor(right - 0.5), as integers
			local c0, c1
			if xa < xb then c0, c1 = -((-xa) // 1) | 0, xb // 1 | 0
			else c0, c1 = -((-xb) // 1) | 0, xa // 1 | 0 end
			if c0 < 0 then c0 = 0 end
			if c1 > w1 then c1 = w1 end
			if c0 ~= sc0 or c1 ~= sc1 then                          -- rows with the same run: one box
				if s0 then box(target, sc0, s0, sc1 - sc0 + 1, r - s0, col) end
				if c1 >= c0 then s0, sc0, sc1 = r, c0, c1 else s0, sc0, sc1 = nil, 0, -1 end
			end
			xa, xb, yy = xa + sa, xb + sb, yy + 1
		end
		if s0 then box(target, sc0, s0, sc1 - sc0 + 1, r1 + 1 - s0, col) end
	end
	-- the framebuffer's box (target: the framebuffer table)
	local function fb_box(t, c0, r, wd, nr, col)
		for rr = r, r + nr - 1 do
			local base = rr * fb_w + 1 + c0
			for c = 0, wd - 1 do t[base + c] = col end
		end
	end

	local function fb_line(x0, y0, x1, y1, col)
		local dx, dy = x1 - x0, y1 - y0
		local steps = max(abs(dx), abs(dy))
		if steps < 1 then steps = 1 end
		steps = ceil(steps)
		for i = 0, steps do
			local c, r = floor(x0 + dx * i / steps), floor(y0 + dy * i / steps)
			if c >= 0 and c < fb_w and r >= 0 and r < fb_h then fb[r * fb_w + c + 1] = col end
		end
	end

	-- send the framebuffer to MAME as the fewest rectangles we can find (horizontal runs merged vertically)
	-- Render quality: always the highest the display can show. The panel is rasterised with one
	-- framebuffer cell per real output pixel: the emulated screen is scaled up by (window height /
	-- emulated height), so each emulated pixel is split into that many cells in each direction.
	local function output_scale()
		local ok, th = pcall(function() return manager.machine.render.ui_target.height end)
		local _, sh = screen_size()
		if ok and th and sh and sh > 0 then return max(1, floor(th / sh)) end
		return 3
	end

	local function fb_emit(x_lo, y_lo, ss)
		local count = 0
		local prev = {}
		local function emit(key_c1, key_c2, col, rs, re)
			dbox(x_lo + key_c1 / ss, y_lo + rs / ss, x_lo + (key_c2 + 1) / ss, y_lo + (re + 1) / ss, col, 0)
			count = count + 1
		end
		for r = 0, fb_h do
			local cur = {}
			if r < fb_h then
				local base = r * fb_w + 1
				local c = 0
				while c < fb_w do
					local col = fb[base + c]
					if col ~= 0 then
						local c2 = c
						while c2 + 1 < fb_w and fb[base + c2 + 1] == col do c2 = c2 + 1 end
						local key = c .. ":" .. c2 .. ":" .. col
						cur[key] = prev[key] or { c, c2, col, r }
						c = c2 + 1
					else
						c = c + 1
					end
				end
			end
			for key, run in pairs(prev) do
				if not cur[key] then emit(run[1], run[2], run[3], run[4], r - 1) end
			end
			prev = cur
		end
		return count
	end

	-- Preferred path: paint the panel into an ARGB bitmap and hand MAME ONE textured quad per frame.
	-- (Thousands of draw_box rectangles per frame at 4K overload the renderer and can crash MAME.)
	local pbm, ptex, pbm_w, pbm_h = nil, nil, 0, 0
	-- reuse of the finished panel image while nothing that shows in it changes (see draw_panel)
	local PCACHE = { key = nil, at = -1e9, quad = nil }
	local retired = {}                      -- textures replaced after a resize, freed a few frames later
	local tex_ok = nil                      -- nil = untested, false = this MAME has no bitmap / texture API
	local function release_panel_texture()
		for _, r in ipairs(retired) do pcall(function() r.tex:free() end) end
		retired = {}
		if ptex then pcall(function() ptex:free() end) end
		pbm, ptex, pbm_w, pbm_h = nil, nil, 0, 0
	end
	local function panel_bitmap(w, h)
		if pbm and pbm_w == w and pbm_h == h then return true end
		local ok = pcall(function()
			local nb = emu.bitmap_argb32(w, h)
			local nt = manager.machine.render:texture_alloc(nb)
			if ptex then retired[#retired + 1] = { tex = ptex, bm = pbm, at = frame_no + 10 } end
			pbm, ptex, pbm_w, pbm_h = nb, nt, w, h
		end)
		return ok
	end
	local bm_box = nil                  -- pbm.plot_box, looked up once per frame (fill_poly's box for the bitmap)
	-- a texture's pixels as numbers (r, g, b per texel; r = -1 where it is magenta = transparent)
	local function tex_rgb(t)
		local T = t.rgb
		if T then return T end
		T = {}
		local px = t.px
		for i = 1, #px, 3 do
			local rr, gg, bb = px:byte(i, i + 2)
			if rr > 240 and gg < 24 and bb > 240 then rr = -1 end
			T[i], T[i + 1], T[i + 2] = rr, gg, bb
		end
		t.rgb = T
		return T
	end
	-- channel value * shade, clamped and floored like argb() does (one table per shade, reused)
	local SHADE_LUT, shade_luts = {}, 0
	local function shade_lut(sh)
		local L = SHADE_LUT[sh]
		if L then return L end
		if shade_luts >= 256 then SHADE_LUT, shade_luts = {}, 0 end
		L = {}
		for v = 0, 255 do L[v] = floor(clamp(v * sh, 0, 255)) end
		SHADE_LUT[sh], shade_luts = L, shade_luts + 1
		return L
	end
	-- a textured quad: pts[1] = image top-left, [2] top-right, [4] bottom-left (mapped as a parallelogram)
	-- (pixels go to plot(first column, row, colour, count), a run of the same colour at a time)
	local function bm_tex(pts, f, W, H, plot)
		local t, sh = f.tex, f.shade
		local p1, p2, p4 = pts[1], pts[2], pts[4]
		local ux, uy = p2[1] - p1[1], p2[2] - p1[2]
		local vx, vy = p4[1] - p1[1], p4[2] - p1[2]
		local det = ux * vy - uy * vx
		if abs(det) < 1e-6 then return end
		local n = #pts
		local miny, maxy = huge, -huge
		for i = 1, n do
			local y = pts[i][2]
			if y < miny then miny = y end
			if y > maxy then maxy = y end
		end
		local tw, th = t.w, t.h
		local T, L = tex_rgb(t), shade_lut(sh)
		local tw1, th1 = tw - 1, th - 1
		local su, sv = tw / det, th / det
		local du, dv = vy * su, -uy * sv                       -- texel steps per pixel along a row
		local r0, r1 = max(ceil(miny - 0.5), 0), min(floor(maxy - 0.5), H - 1)
		for r = r0, r1 do
			local yy = r + 0.5
			local xmin, xmax = huge, -huge
			for i = 1, n do
				local a, b = pts[i], pts[i % n + 1]
				local ay, by = a[2], b[2]
				if (ay <= yy and by > yy) or (by <= yy and ay > yy) then
					local x = a[1] + (yy - ay) / (by - ay) * (b[1] - a[1])
					if x < xmin then xmin = x end
					if x > xmax then xmax = x end
				end
			end
			if xmin <= xmax then
				local c0, c1 = max(ceil(xmin - 0.5), 0), min(floor(xmax - 0.5), W - 1)
				-- texel coordinates at the first pixel's centre, stepped along the row
				local dy, dx = yy - p1[2], c0 + 0.5 - p1[1]
				local fu, fv = (dx * vy - dy * vx) * su, (ux * dy - uy * dx) * sv
				local run, rcol = nil, 0                            -- a run of same-colour pixels: one call
				for c = c0, c1 do
					local tx, ty = fu // 1, fv // 1                       -- (floor, as floats: table keys
					fu, fv = fu + du, fv + dv                             -- normalise them to integers)
					if tx < 0 then tx = 0 elseif tx > tw1 then tx = tw1 end
					if ty < 0 then ty = 0 elseif ty > th1 then ty = th1 end
					local i = (ty * tw + tx) * 3 + 1
					local rr = T[i]
					if rr >= 0 then                                     -- (magenta = transparent)
						local col = 0xFF000000 | (L[rr] << 16) | (L[T[i + 1]] << 8) | L[T[i + 2]]
						if col ~= rcol or not run then
							if run then plot(run, r, rcol, c - run) end
							run, rcol = c, col
						end
					elseif run then
						plot(run, r, rcol, c - run) run = nil
					end
				end
				if run then plot(run, r, rcol, c1 + 1 - run) end
			end
		end
	end
	local function bm_line(x0, y0, x1, y1, col)
		local dx, dy = x1 - x0, y1 - y0
		local steps = max(1, ceil(max(abs(dx), abs(dy))))
		for i = 0, steps do
			local c, r = floor(x0 + dx * i / steps), floor(y0 + dy * i / steps)
			if c >= 0 and c < pbm_w and r >= 0 and r < pbm_h then pbm:plot(c, r, col) end
		end
	end
	local function flush_faces_texture(cxp, cyp, x_lo, y_lo, x_hi, y_hi)
		local ss = output_scale()
		local w, h = max(1, floor((x_hi - x_lo) * ss)), max(1, floor((y_hi - y_lo) * ss))
		if not panel_bitmap(w, h) then return false end
		for i = #retired, 1, -1 do
			if frame_no >= retired[i].at then pcall(function() retired[i].tex:free() end) table.remove(retired, i) end
		end
		pbm:fill(0)                                         -- fully transparent
		if not faces_sorted then table.sort(faces, face_order) end
		local outline = argb(0xFF, 12, 14, 18)
		local fl = cfg.focal_scale * cam.focal
		local ox, oy = (cxp - x_lo) * ss, (cyp - y_lo) * ss
		local fs = fl * ss
		local xs, ys = {}, {}
		bm_box = pbm.plot_box
		local plot_tex = function(c, r, col, len) bm_box(pbm, c, r, len, 1, col) end
		for _, f in ipairs(faces) do
			local vs = f.vs
			local n = #vs
			for i = 1, n do
				local v = vs[i]
				local iz = 1 / v[3]
				xs[i], ys[i] = ox + fs * v[1] * iz, oy - fs * v[2] * iz
			end
			local pp
			if f.tex or cfg.outline then
				pp = {}
				for i = 1, n do pp[i] = { xs[i], ys[i] } end
			end
			if f.tex then
				bm_tex(pp, f, pbm_w, pbm_h, plot_tex)
			else
				local r, g, b = f.r // 1, f.g // 1, f.b // 1                -- (argb, inlined)
				if r > 255 then r = 255 end
				if g > 255 then g = 255 end
				if b > 255 then b = 255 end
				fill_poly(xs, ys, n, pbm_w, pbm_h, bm_box, pbm, 0xFF000000 | (r | 0) << 16 | (g | 0) << 8 | (b | 0))
			end
			if cfg.outline then
				for i = 1, #pp do
					local a, b = pp[i], pp[i % #pp + 1]
					bm_line(a[1], a[2], b[1], b[2], outline)
				end
			end
		end
		local sw, sh = screen_size()
		PCACHE.quad = { x_lo / sw, y_lo / sh, x_hi / sw, y_hi / sh }
		scr.container:draw_quad(ptex, x_lo / sw, y_lo / sh, x_hi / sw, y_hi / sh)
		last_rects = 1
		return true
	end

	local function flush_faces(cxp, cyp, x_lo, y_lo, x_hi, y_hi)
		if tex_ok ~= false then
			local ok, done = pcall(flush_faces_texture, cxp, cyp, x_lo, y_lo, x_hi, y_hi)
			if ok and done then tex_ok = true return end
			if tex_ok == nil then tex_ok = false end            -- never worked: use the rectangles from now on
		end
		local ss = output_scale()
		local w, h = floor((x_hi - x_lo) * ss), floor((y_hi - y_lo) * ss)
		fb_clear(w, h)
		if not faces_sorted then table.sort(faces, face_order) end
		local outline = argb(0xFF, 12, 14, 18)
		for _, f in ipairs(faces) do
			local pp, xs, ys = {}, {}, {}
			for i, v in ipairs(f.vs) do
				local z = v[3]
				xs[i] = (cxp + cfg.focal_scale * cam.focal * v[1] / z - x_lo) * ss
				ys[i] = (cyp - cfg.focal_scale * cam.focal * v[2] / z - y_lo) * ss
				pp[i] = { xs[i], ys[i] }
			end
			if f.tex then                                       -- textures in the rectangle fallback too
				bm_tex(pp, f, fb_w, fb_h, function(c, r, col, len) fb_box(fb, c, r, len, 1, col) end)
			else
				fill_poly(xs, ys, #xs, fb_w, fb_h, fb_box, fb, argb(0xFF, f.r, f.g, f.b))
			end
			if cfg.outline then
				for i = 1, #pp do
					local a, b = pp[i], pp[i % #pp + 1]
					fb_line(a[1], a[2], b[1], b[2], outline)
				end
			end
		end
		last_rects = fb_emit(x_lo, y_lo, ss)
	end

	---------------------------------------------------------------------------
	-- scene
	---------------------------------------------------------------------------
	-- rectangular bar between two points (used for the arms and accessory brackets)
	local function emit_bar(x0, y0, z0, x1, y1, z1, hw, hh, rgb, yaw)
		yaw = yaw or 0
		local dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
		local len = sqrt(dx * dx + dy * dy + dz * dz)
		if len < 1e-6 then return end
		dx, dy, dz = dx / len, dy / len, dz / len
		local ux, uy, uz = 0, 1, 0
		if abs(dy) > 0.95 then ux, uy, uz = 1, 0, 0 end
		local sx, sy, sz = dy * uz - dz * uy, dz * ux - dx * uz, dx * uy - dy * ux
		local sl = sqrt(sx * sx + sy * sy + sz * sz)
		sx, sy, sz = sx / sl, sy / sl, sz / sl
		local vx, vy, vz = sy * dz - sz * dy, sz * dx - sx * dz, sx * dy - sy * dx
		local c = {}
		for i = 0, 7 do
			local a = (i & 4) ~= 0 and 1 or -1
			local b = (i & 2) ~= 0 and 1 or -1
			local e = (i & 1) ~= 0
			local px, py, pz = e and x1 or x0, e and y1 or y0, e and z1 or z0
			local rx, rz = rot_y(px + sx * hw * a + vx * hh * b, pz + sz * hw * a + vz * hh * b, yaw)
			c[i + 1] = { rx, py + sy * hw * a + vy * hh * b, rz }
		end
		local mx, mz = rot_y((x0 + x1) / 2, (z0 + z1) / 2, yaw)
		emit_hexa(c, mx, (y0 + y1) / 2, mz, rgb)
	end

	---------------------------------------------------------------------------
	-- scene  (units: 1 = about 1.85 cm; the real unit is 24 cm tall -> ~13 units)
	---------------------------------------------------------------------------
	local COL = {
		red = { 208, 44, 44 },    blue = { 56, 100, 218 },  spinner = { 190, 160, 60 },
		post = { 168, 168, 174 }, magenta = { 176, 40, 100 }, glass = { 16, 16, 26 },
		grey = { 150, 152, 158 },
	}
	-- Colour roles (taken from photos of the real units):
	--   shell : main plastic (head, mast, shoulder housing, base)
	--   band  : bottom band of the base
	--   arm   : the arm bars
	--   hand  : smoked-black hands and head visor
	--   lens  : eye lenses
	--   table : the turntable disc on top of the base
	--   label : the strip on the front of the base
	--   claw  : the snap-on gripper claws / grip pads
	local PALETTES = {
		-- NES R.O.B. (NES-012): light grey, mid-grey arms, darker grey base band, smoked-black visor + hands
		us = {
			shell = { 198, 199, 196 }, band = { 122, 123, 128 }, arm = { 112, 114, 120 },
			hand = { 34, 34, 38 }, trim = { 26, 26, 30 }, lens = { 70, 72, 80 },
			table = { 170, 172, 178 }, label = { 196, 36, 40 }, claw = { 92, 92, 100 },
			labelbg = { 40, 44, 52 }, post = { 176, 177, 180 },
		},
		-- Famicom Robot (HVC-012): white, maroon arms and base band, smoked-black visor + hands
		famicom = {
			shell = { 238, 236, 228 }, band = { 128, 28, 38 }, arm = { 132, 28, 40 },
			hand = { 38, 34, 38 }, trim = { 28, 26, 28 }, lens = { 72, 70, 76 },
			table = { 226, 224, 216 }, label = { 128, 28, 38 }, claw = { 132, 28, 40 },
			labelbg = { 236, 232, 222 }, post = { 222, 220, 212 },
		},
	}
	apply_palette = function(kind)
		local v = cfg.variant
		if v == nil or v == "auto" then v = kind end
		local pal = PALETTES[v == "famicom" and "famicom" or "us"]
		for k, c in pairs(pal) do COL[k] = c end
	end
	apply_palette(machine_kind)

	local function lighten(c, k) return { clamp(c[1] + k, 0, 255), clamp(c[2] + k, 0, 255), clamp(c[3] + k, 0, 255) } end

	local function station_pos(s)
		local a = (s - 2) * 60 * RAD
		return HOLD_R * sin(a), HOLD_R * cos(a)
	end

	-- height of the hands. Stop 0 holds a hand level with the bottom Stack-Up disk; each stop is one disk higher.
	local function arm_y() return HAND_Y0 + R.h * ARM_STEP end

	-- Gyromite gyro (from photos): a dark red disc with a black / white checkered band round the rim,
	-- a thick light grey spindle on top (R.O.B.'s grippers hold it) and a short tip underneath.
	local GYRO_R, GYRO_RIN, GYRO_POST = 1.9, 1.68, SPINDLE_LEN      -- rim band 0.22 wide
	-- Gyro geometry in its own frame, measured from the tip (v up the spin axis): spindle below the
	-- disc 0..T (T = GYRO_TIPL), disc T..T+0.22, hub, spindle to GYRO_TOP. It can be drawn tilted by th (radians) toward
	-- horizontal direction dir (0 = +z), which is how a toppled / carried gyro is shown.
	local GYRO_TOP = GYRO_TIPL + 0.22 + GYRO_POST
	local function emit_gyro_t(tx, ty, tz, th, dir, g, vmin)   -- vmin: leg hidden below this (inside a tower)
		local ang = g.phase
		local ct, st = cos(th), sin(th)
		local dx, dz = sin(dir), cos(dir)    -- tilt direction (u axis)
		local wx, wz = cos(dir), -sin(dir)   -- the other horizontal axis (w)
		local function P(u, v, w)            -- local -> world
			local u2, v2 = u * ct + v * st, -u * st + v * ct
			return { tx + u2 * dx + w * wx, ty + v2, tz + u2 * dz + w * wz }
		end
		local function ring(r, v, a) return P(r * cos(a), v, r * sin(a)) end
		local function centre(v) local q = P(0, v, 0) return q[1], q[2], q[3] end
		-- a cylinder / cone section between heights v0, v1 (radii r0, r1)
		local function section(r0, r1, v0, v1, n, side, top)
			local cx, cy, cz = centre((v0 + v1) / 2)
			local lo, hi = {}, {}
			for i = 0, n - 1 do
				local a = ang + i * 2 * math.pi / n
				lo[i + 1], hi[i + 1] = ring(r0, v0, a), ring(r1, v1, a)
			end
			emit_rings(lo, hi, cx, cy, cz, side, top and hi, top)
		end
		local T, D = GYRO_TIPL, GYRO_TIPL + 0.22                                            -- disc bottom / top
		-- the spindle runs on through the disc: grey below it too, narrowing to a short point at the tip
		vmin = vmin or 0
		-- an upright gyro's leg is under the disc: pushed back in the depth sort so the disc always covers
		-- it (a short stub left above a holder / tower would otherwise sort in front of the disc's top)
		local dv, fb = decal_any_view, face_bias
		if abs(th) < 0.05 then decal_any_view, face_bias = true, -0.45 end
		if vmin < 0.18 then section(0.1, 0.3, 0, 0.18, 10, { 206, 206, 200 }) end
		if vmin < T then section(0.3, 0.3, max(0.18, vmin), T, 10, { 222, 222, 216 }) end
		decal_any_view, face_bias = dv, fb
		-- disc: dark rim wall and underside, checkered band and dark red centre on top
		local n = 32                         -- 16 rim segments, alternating black and white (2 facets each)
		-- spinning fast the checks would strobe at 60 fps: they blur toward an even grey instead
		local bl = clamp((g.spin - 0.25) / 0.35, 0, 1) * 0.92
		local band_k = { floor(30 + 103 * bl), floor(30 + 103 * bl), floor(34 + 102 * bl) }
		local band_w = { floor(236 - 103 * bl), floor(236 - 103 * bl), floor(238 - 102 * bl) }
		local cx, cy, cz = centre(T + 0.11)
		local bot = {}
		for i = 0, n - 1 do
			local a0, a1 = ang + i * 2 * math.pi / n, ang + (i + 1) * 2 * math.pi / n
			add_face({ ring(GYRO_R, T, a0), ring(GYRO_R, T, a1), ring(GYRO_R, D, a1), ring(GYRO_R, D, a0) },
				cx, cy, cz, { 214, 214, 218 })
			add_face({ ring(GYRO_RIN, D, a0), ring(GYRO_R, D, a0), ring(GYRO_R, D, a1), ring(GYRO_RIN, D, a1) },
				cx, cy, cz, ((i // 2) % 2 == 0) and band_k or band_w)
			bot[#bot + 1] = ring(GYRO_R, T, a0)
		end
		add_face(bot, cx, cy, cz, { 60, 60, 66 })
		local c = {}
		for i = 0, 31 do c[#c + 1] = ring(GYRO_RIN, D, ang + i * 2 * math.pi / 32) end
		add_face(c, cx, cy, cz, { 138, 32, 42 })
		if TEX.gyro then
			-- the label on the red top: "Gyro" along one edge, "Nintendo" along the other (photos);
			-- a square image whose corners outside the circle are transparent
			local saved = face_bias
			face_bias = saved + 0.25
			local rq = GYRO_RIN * math.sqrt(2)
			local q4 = {}
			for k = 0, 3 do q4[k + 1] = ring(rq, D + 0.004, ang + (135 - k * 90) * math.pi / 180) end
			add_face(q4, cx, cy - 1, cz, { 138, 32, 42 }, TEX.gyro)
			face_bias = saved
		end
		section(0.3, 0.3, D, D + 0.18, 10, { 120, 28, 38 }, { 150, 36, 46 })                 -- red hub
		section(0.3, 0.3, D, GYRO_TOP, 10, { 222, 222, 216 }, { 236, 236, 230 })        -- spindle tube
		section(0.36, 0.36, GYRO_TOP - 0.1, GYRO_TOP, 12, { 214, 214, 208 }, { 238, 238, 232 })  -- ring at the top
		do                                                                                   -- tube bore
			local hcx, hcy, hcz = centre(GYRO_TOP + 0.01)
			local h = {}
			for i = 0, 9 do h[#h + 1] = ring(0.15, GYRO_TOP + 0.01, ang + i * 2 * math.pi / 10) end
			add_face(h, hcx, hcy, hcz, { 60, 60, 64 })
		end
	end
	-- lowest point of a gyro tilted by th, relative to its tip (for resting it on the floor)
	local GYRO_HULL = { { 0, 0 }, { GYRO_R, GYRO_TIPL }, { -GYRO_R, GYRO_TIPL }, { GYRO_R, GYRO_TIPL + 0.22 }, { -GYRO_R, GYRO_TIPL + 0.22 },
		{ 0.27, GYRO_TOP }, { -0.27, GYRO_TOP } }
	gyro_lowest = function(th)
		local lo = huge
		for _, q in ipairs(GYRO_HULL) do lo = min(lo, -q[1] * sin(th) + q[2] * cos(th)) end
		return lo
	end
	-- upright gyro whose disc bottom is at gb (yaw rotates its position about the robot's axis)
	local function emit_gyro(gx, gb, gz, g, yaw, s)          -- s: the station it stands in / over
		local x, z = rot_y(gx, gz, yaw)
		-- its leg goes down into a holder / the spinner's tower (hidden below the top); on a button it
		-- stands on the cap on its tip
		local role = s and STATIONS[s] and STATIONS[s].role
		local top = (role == "spinner" and seat_of(s) - 0.12) or (role == "dock" and G_SEAT) or nil
		local vmin = top and top - (gb - GYRO_TIPL)
		emit_gyro_t(x, gb - GYRO_TIPL, z, 0, 0, g, vmin)
	end

	-- Stack-Up block (from a 3D model of the real one, and photos): a 40 mm cylinder with a conical recess
	-- in its top and a matching cone underneath; a block's cone seats in the recess of the block below (or
	-- in the stand's centre hole). Silver print on the top (block_label.ppm). Built in the block's own
	-- frame -- P(u, v, w): u, w across, v up from the block's bottom -- so the same geometry serves a
	-- standing / carried block and a tumbling one.
	--   top  : show the recess (a block with another on it has the next block's cone in it: drawn flat)
	--   cone : show the cone underneath (a block standing on something has it hidden in there)
	local emit_disc_t, emit_block
	do                                   -- (its helpers stay local to this block: the main chunk is at Lua's local limit)
	local function block_mesh(P, c, top, cone)
		local REC_R, REC_F, REC_D = 0.64, 0.3, 0.38      -- recess: radius at the top, floor radius, depth
		local CONE_R0, CONE_R1, CONE_H = 0.6, 0.28, 0.38 -- cone: radius at the block's bottom, at its tip, height
		local H, n = DISC_H, 18
		local mid = P(0, H / 2, 0)
		local function ring(r, v)
			local q = {}
			for i = 0, n - 1 do local a = i * 2 * math.pi / n q[i + 1] = P(r * cos(a), v, r * sin(a)) end
			return q
		end
		emit_rings(ring(DISC_R, 0), ring(DISC_R, H), mid[1], mid[2], mid[3], c)    -- outer wall
		-- top: a flat ring round a conical recess, or flat all over
		if top then
			local rim, rh, fl = ring(DISC_R, H), ring(REC_R, H), ring(REC_F, H - REC_D)
			for i = 1, n do
				local j = i % n + 1
				add_face({ rim[i], rim[j], rh[j], rh[i] }, mid[1], mid[2], mid[3], lighten(c, 26))
				local am = (i - 0.5) * 2 * math.pi / n
				local o = P(3 * REC_R * cos(am), H - REC_D / 2, 3 * REC_R * sin(am))   -- outside: the wall faces in
				add_face({ rh[i], rh[j], fl[j], fl[i] }, o[1], o[2], o[3], lighten(c, -12))
			end
			local below = P(0, H - REC_D - 0.5, 0)
			add_face(fl, below[1], below[2], below[3], lighten(c, -24))                   -- recess floor
		else
			add_face(ring(DISC_R, H), mid[1], mid[2], mid[3], lighten(c, 26))
		end
		-- bottom: a flat ring round the cone, or flat all over
		if cone then
			local bo, b0, b1 = ring(DISC_R, 0), ring(CONE_R0, 0), ring(CONE_R1, -CONE_H)
			local ax = P(0, -CONE_H / 2, 0)
			for i = 1, n do
				local j = i % n + 1
				add_face({ bo[i], bo[j], b0[j], b0[i] }, mid[1], mid[2], mid[3], lighten(c, -20))
				add_face({ b0[i], b0[j], b1[j], b1[i] }, ax[1], ax[2], ax[3], lighten(c, -15))
			end
			add_face(b1, mid[1], mid[2], mid[3], lighten(c, -20))                         -- the cone's flat tip
		else
			add_face(ring(DISC_R, 0), mid[1], mid[2], mid[3], lighten(c, -20))
		end
		-- the silver print on the top ring: a partial ring, "BLOCK" on the far side, "Nintendo" on the near
		if top and TEX.block then
			local dv, fb = decal_any_view, face_bias
			decal_any_view, face_bias = true, 0.7               -- (faces turned away are culled anyway)
			local rq = DISC_R * math.sqrt(2)
			local q4 = {}
			for k = 0, 3 do
				local a = (225 - k * 90) * RAD                  -- image top-left, top-right, bottom-right, bottom-left
				q4[k + 1] = P(rq * cos(a), H + 0.015, rq * sin(a))
			end
			add_face(q4, mid[1], mid[2], mid[3], c, TEX.block)
			decal_any_view, face_bias = dv, fb
		end
	end

	-- a tumbling / lying block, centred at (cx, cy, cz), tilted by th toward horizontal direction dir
	function emit_disc_t(cx, cy, cz, th, dir, name)
		local ct, st = cos(th), sin(th)
		local dx, dz, wx, wz = sin(dir), cos(dir), cos(dir), -sin(dir)
		local function P(u, v, w)
			v = v - DISC_H / 2                                  -- (from the block's centre)
			local u2, v2 = u * ct + v * st, -u * st + v * ct
			return { cx + u2 * dx + w * wx, cy + v2, cz + u2 * dz + w * wz }
		end
		block_mesh(P, BLOCK_COL[name] or { 200, 200, 200 }, true, true)
	end

	-- a standing / carried block with its bottom at y0 (yaw turns its position about the robot's axis)
	function emit_block(cx, y0, cz, name, yaw, top, cone)
		yaw = yaw or 0
		local function P(u, v, w)
			local x, z = rot_y(cx + w, cz + u, yaw)
			return { x, y0 + v, z }
		end
		block_mesh(P, BLOCK_COL[name] or { 200, 200, 200 }, top, cone)
	end
	end

	-- The parts of the scene that never move (the base, the head, the accessories' posts / spinner / trays)
	-- are built once and their faces kept: while the camera, colours and accessories stay the same, each
	-- frame only builds the moving parts and adds the kept faces to them. Kept faces are numbered from 1,
	-- each frame's own from 1e6 on, so faces at the same depth always paint in the same order.
	local SCACHE = { key = nil, faces = nil, parts = {} }
	local function build_scene()
		cam_prepare()
		local skey = table.concat({ game, tostring(STATIONS), tostring(COL.shell), tostring(COL.post), cam.yaw, cam.pitch,
			cam.dist, cam.ty, cam.tx or 0, cam.tz or 0, tostring(cfg.mirror_lr) }, "|")
		local build_static = SCACHE.key ~= skey
		local dfaces, sfaces = {}, build_static and {} or nil
		-- while a static part is being built its faces go to the kept list (only when it is being rebuilt)
		local function static_begin()
			if build_static then faces = sfaces face_seq = #sfaces end
			return build_static
		end
		local function static_end() faces = dfaces face_seq = 1000000 + #dfaces end
		static_end()
		-- A moving part (the body, the buttons) whose pose is the same as last frame's (often: a gyro
		-- spinning or the LED blinking redraws the panel while the robot stands still) reuses last frame's
		-- faces, renumbered in turn, instead of building them again: part_reuse(name, pose) says whether
		-- it did; if not, the part is built and part_keep(name) keeps its faces.
		local parts = SCACHE.parts
		local function part_reuse(name, pose)
			local P = parts[name]
			local key = skey .. "|" .. pose
			if P and P.key == key then
				for _, f in ipairs(P.faces) do
					face_seq = face_seq + 1
					f.seq = face_seq
					faces[#faces + 1] = f
				end
				return true
			end
			parts[name] = { key = key, from = #faces + 1 }
			return false
		end
		local function part_keep(name)
			local P = parts[name]
			P.faces = table.move(faces, P.from, #faces, 1, {})
		end
		front_view = cos(cam.yaw) < 0
		local lx, ly, lz = light_world[1], light_world[2], light_world[3]
		local vx, vy, vz = w2v(lx, ly + cam.ty, lz)
		local ox, oy, oz = w2v(0, cam.ty, 0)
		local dx, dy, dz = vx - ox, vy - oy, vz - oz
		local dl = sqrt(dx * dx + dy * dy + dz * dz)
		light_view = { dx / dl, dy / dl, dz / dl }

		local shell, trim = COL.shell, COL.trim
		local SEAT = (game == "stackup") and STACK_SEAT or G_SEAT       -- top of the accessory posts / resting height

		-- which stations currently hold a gyro
		local occupied = {}
		for _, g in pairs(gyros) do if g.loc ~= "held" then occupied[g.station] = g end end

		local post = COL.post
		local tube = lighten(COL.post, 26)                     -- the off-white tube posts (photos)
		-- triangular bracket (photos): held a little off the floor (bottom edge at yb) between the base
		-- wall and the side of the post -- the post stands on the floor, the bracket does not -- its top
		-- edge sloping from the height of the base (yt0, at the base wall) down to the post (yt1); flat
		-- sides, lattice along the top
		local function bracket(x0, z0, x1, z1, yb, yt0, yt1, col)
			local sf = split_faces
			split_faces = true                                   -- long faces: cut up so they sort cleanly
			local HW, WT = 0.2, 0.05
			local dx, dz = x1 - x0, z1 - z0
			local L = sqrt(dx * dx + dz * dz)
			dx, dz = dx / L, dz / L
			local nx, nz = -dz, dx
			local function P(u, y, s2) return { x0 + dx * u + nx * s2, y, z0 + dz * u + nz * s2 } end
			local function top(u) return yt0 + (yt1 - yt0) * clamp(u / L, 0, 1) end
			local function slab(poly, s0, s1, c)                     -- (u, y) polygon across s0 .. s1
				local A, B = {}, {}
				local cu, cy = 0, 0
				for i, q in ipairs(poly) do A[i], B[i] = P(q[1], q[2], s0), P(q[1], q[2], s1) cu, cy = cu + q[1], cy + q[2] end
				cu, cy = cu / #poly, cy / #poly
				local m = P(cu, cy, (s0 + s1) / 2)
				emit_rings(A, B, m[1], m[2], m[3], c, A, c, B, c)
			end
			local side = { { 0, yb }, { L, yb }, { L, yt1 }, { 0, yt0 } }
			slab(side, -HW, -HW + WT, col)                                           -- flat sides
			slab(side, HW - WT, HW, col)
			slab({ { 0, yb }, { L, yb }, { L, yb + 0.12 }, { 0, yb + 0.12 } }, -HW + WT, HW - WT, col)  -- bottom
			slab({ { 0, yt0 - 0.3 }, { L, yt1 - 0.3 }, { L, yt1 - 0.26 }, { 0, yt0 - 0.26 } }, -HW + WT, HW - WT, lighten(col, -60))  -- web
			local n = max(2, floor(sqrt(L * L + (yt0 - yt1) ^ 2) / 0.45 + 0.5))
			local si = HW - WT
			for k = 0, n - 1 do                                                       -- struts along the top
				local ua, ub = L * k / n, L * (k + 1) / n
				local s0, s1 = (k % 2 == 0) and -si or si, (k % 2 == 0) and si or -si
				local q = { { ua, s0 }, { ua + 0.09, s0 }, { ub, s1 }, { ub - 0.09, s1 } }
				local lo, hi = {}, {}
				for i, p2 in ipairs(q) do
					lo[i] = P(p2[1], top(p2[1]) - 0.26, p2[2])
					hi[i] = P(p2[1], top(p2[1]), p2[2])
				end
				local m = P((ua + ub) / 2, top((ua + ub) / 2) - 0.13, 0)
				emit_rings(lo, hi, m[1], m[2], m[3], lighten(col, -4))
				add_face(hi, m[1], m[2] - 1, m[3], lighten(col, 2))
			end
			split_faces = sf
		end
		-- a round tube post standing on the floor
		local FOOT_H = 1.5                                     -- the post's foot cylinder (stands on the floor; the bracket
		                                                       -- meets its side). Holders and buttons alike: a pressed
		                                                       -- button's cap comes down to just above it
		local function tube_post(px, pz, y1, r, open_top)
			local fh = FOOT_H
			-- foot: a wider cylinder at the bottom, where the bracket joins, the same plastic as the tube
			-- (cut into small pieces so it sorts cleanly against the bracket)
			local sf = split_faces
			split_faces = true
			emit_prism(px, fh / 2, pz, r + 0.15, fh, 14, tube, 0, 0, tube)
			split_faces = sf
			-- the tube starts on top of the foot (nothing runs through the foot)
			if not open_top then emit_prism(px, (fh + y1) / 2, pz, r, y1 - fh, 14, tube, 0, 0, lighten(tube, 6)) return end
			-- sides only: the top is hidden under whatever sits on it (and must not sort through it)
			for i = 0, 13 do
				local a0, a1 = i * 2 * math.pi / 14, (i + 1) * 2 * math.pi / 14
				add_face({ { px + r * cos(a0), fh, pz + r * sin(a0) }, { px + r * cos(a1), fh, pz + r * sin(a1) },
					{ px + r * cos(a1), y1, pz + r * sin(a1) }, { px + r * cos(a0), y1, pz + r * sin(a0) } }, px, (fh + y1) / 2, pz, tube)
			end
		end
		-- a cup open at the top (outer radius ro, bore ri, floor at yf, rim at y1)
		-- a cup open at the top (outer radius ro, bore ri, floor at yf, rim at y1); rh > 0 cuts a hole of
		-- that radius through the floor
		local function cup(px, pz, y0, yf, y1, ro, ri, col, inner, rh)
			local n = 20
			local lo, md, hi, inn, fl, hf, hb = {}, {}, {}, {}, {}, {}, {}
			for i = 0, n - 1 do
				local q = i * 2 * math.pi / n
				local cq, sq = cos(q), sin(q)
				lo[i + 1] = { px + ro * cq, y0, pz + ro * sq }
				md[i + 1] = { px + ro * cq, yf, pz + ro * sq }
				hi[i + 1] = { px + ro * cq, y1, pz + ro * sq }
				inn[i + 1] = { px + ri * cq, y1, pz + ri * sq }
				fl[i + 1] = { px + ri * cq, yf, pz + ri * sq }
				if rh then
					hf[i + 1] = { px + rh * cq, yf, pz + rh * sq }
					hb[i + 1] = { px + rh * cq, y0, pz + rh * sq }
				end
			end
			for i = 1, n do
				local j = i % n + 1
				local q = (i - 0.5) * 2 * math.pi / n
				local ox, oz = px + 3 * ro * cos(q), pz + 3 * ro * sin(q)                         -- well outside the wall
				-- outer wall in two bands, split at the floor (what sits in the cup sorts against the upper one)
				add_face({ lo[i], lo[j], md[j], md[i] }, px, (y0 + yf) / 2, pz, col)
				add_face({ md[i], md[j], hi[j], hi[i] }, px, (yf + y1) / 2, pz, col)
				add_face({ hi[i], hi[j], inn[j], inn[i] }, px, y1 - 1, pz, lighten(col, 10))      -- rim: faces up
				add_face({ fl[i], fl[j], inn[j], inn[i] }, ox, (yf + y1) / 2, oz, inner or lighten(col, -20))  -- bore: faces in
				if rh then
					add_face({ hf[i], hf[j], fl[j], fl[i] }, px, yf - 1, pz, inner or lighten(col, -28))         -- floor ring
					add_face({ hb[i], hb[j], hf[j], hf[i] }, ox, (y0 + yf) / 2, oz, lighten(col, -45))           -- hole wall
					add_face({ lo[i], lo[j], hb[j], hb[i] }, px, y0 + 1, pz, lighten(col, -16))                  -- underside ring
				end
			end
			if not rh then
				add_face(lo, px, y0 + 1, pz, lighten(col, -16))                                     -- underside: faces down
				add_face(fl, px, yf - 1, pz, inner or lighten(col, -28))                            -- floor: faces up
			end
		end

		-- ACCESSORIES (from photos of complete sets)
		--   dock    : Gyromite gyro holder: an off-white tube post on the floor with a black cup on top,
		--             clipped to a base slot by a lattice bracket
		--   tray    : Gyromite red / blue button: a flat red / blue cap on an off-white tube post; a gyro
		--             standing on it pushes it down (the controller holder and its lattice are left out)
		--   spinner : Gyromite motor box with a white tower; the gyro's leg drops into its black cup
		--   pad     : Stack-Up tray: a flat round platform with a centre hole, held out from a base slot on
		--             a diagonal strut
		-- R.O.B.'s base outline (plan view, see BASE below)
		local base_p = { { 2.35, 4.55 }, { 4.76, 0.43 }, { 4.76, -0.89 }, { 2.35, -4.55 },
			{ -2.35, -4.55 }, { -4.76, -0.89 }, { -4.76, 0.43 }, { -2.35, 4.55 } }
		-- distance from the centre to the edge of a convex profile along direction a (0 = +Z, front)
		local function prof_reach(prof, a)
			local dx, dz = sin(a), cos(a)
			local best = huge
			for i = 1, #prof do
				local p, q = prof[i], prof[i % #prof + 1]
				local ex, ez = q[1] - p[1], q[2] - p[2]
				local den = dx * ez - dz * ex
				if abs(den) > 1e-9 then
					local t = (p[1] * ez - p[2] * ex) / den
					local u = (p[1] * dz - p[2] * dx) / den
					if t > 0 and u >= -1e-6 and u <= 1 + 1e-6 and t < best then best = t end
				end
			end
			return best
		end
		-- (two passes: the fixed accessories, kept between frames; the red / blue buttons, which move)
		for pass = 1, 2 do
		local fixed = pass == 1
		local pose = string.format("%.17g|%.17g", R.press.red or 0, R.press.blue or 0)
		if fixed and static_begin() or not fixed and not part_reuse("buttons", pose) then
		for s = 0, 4 do
			local st = STATIONS[s]
			if st and ((st.role ~= "tray") == fixed) then
				local px, pz = station_pos(s)
				local a = (s - 2) * 60 * RAD
				local sa, ca = sin(a), cos(a)
				local r0 = prof_reach(base_p, a)               -- base wall along this station's direction
				if st.role == "dock" or st.role == "tray" then
					-- arm (photos of the set): a straight lattice beam rising from the foot of the post up
					-- into the slot in the base wall
					local fr = 0.45 + 0.15 - 0.02                            -- meets the foot's surface (does not run into it)
					bracket(r0 * sa, r0 * ca, px - fr * sa, pz - fr * ca, 0.3, 2.2, FOOT_H - 0.1, tube)
				end
				if st.role == "tray" then
					local col = COL[st.color]
					-- button (photos): a white post like a holder's with a flat red / blue cap on top
					local top = seat_of(s, "tray") - GYRO_TIPL                          -- top of the cap (the gyro's tip stands on it)
					tube_post(px, pz, top - 0.3, 0.45, true)                            -- (its top is under the cap)
					emit_prism(px, top - 0.15, pz, 0.86, 0.3, 24, col, 0, 0, lighten(col, 18))                -- cap
				elseif st.role == "pad" then
					local stand = lighten(post, 18)
					local cyb = SEAT - 0.25                                   -- underside of the platform
					-- flat-topped platform (the block sits on it) with a hole through the middle
					cup(px, pz, SEAT - 0.25, SEAT, SEAT, 1.45, 1.45, stand, lighten(stand, 8), 0.82)
					-- a smaller ring underneath the platform
					cup(px, pz, cyb - 0.55, cyb, cyb, 1.05, 1.05, lighten(stand, -6), lighten(stand, -6), 0.82)
					-- arm: a plate standing on edge, from the slot low on the base wall up to the ring
					local rs, re = r0 + 0.06, HOLD_R - 0.95
					emit_bar(rs * sa, 0.75, rs * ca, re * sa, cyb - 0.3, re * ca, 0.08, 0.3, stand, 0)
					emit_box(0, 1.25, r0 + 0.08, 0.8, 0.55, 0.08, lighten(stand, -10), a)               -- tab in the slot
				elseif st.role == "spinner" then
					-- Spinner (photos of the set): a two-tone box -- darker grey body, lighter top with a raised
					-- ridge -- with a white tower and a black cup near the end facing the robot. The CAUTION
					-- sticker is on a long side, low down. A clip joins the box to its slot in the base.
					-- Local frame: z radial (out from the robot), x across.
					local BW = 3.2                                           -- width
					local BH0, BH1 = 2.5, 2.95                               -- walls top, top face (taller than the base: photos)
					local zi = r0 - 0.15                                     -- inner end: against (into) the base wall
					local zo = HOLD_R + 1.2                                  -- outer end, past the tower
					local BL = zo - zi
					local bz, hz = (zi + zo) / 2, BL / 2
					-- shape (photos): rounded vertical corners; a darker lower band, a lighter upper part, a
					-- wide sloped bevel along the top edges up to a smaller top face, two raised pads on the
					-- top at the end nearest the robot
					local body = lighten(post, -22)
					local upper = lighten(post, 10)
					local function outline(inset)
						local q = rrect(BW / 2 - inset, hz - inset, max(0.06, 0.32 - inset * 0.4), 3)
						local out = {}
						for i, p2 in ipairs(q) do out[i] = { p2[1], p2[2] + bz } end
						return out
					end
					local o0 = outline(0)
					local ccx, ccz = rot_y(0, bz, a)
					-- loft between two outlines (same point count) at heights y0 / y1; optional top face
					local function loft(p0, y0, p1, y1, col, top_col)
						local A, B = {}, {}
						for i = 1, #p0 do
							local x0, z0 = rot_y(p0[i][1], p0[i][2], a)
							local x1, z1 = rot_y(p1[i][1], p1[i][2], a)
							A[i], B[i] = { x0, y0, z0 }, { x1, y1, z1 }
						end
						emit_rings(A, B, ccx, (y0 + y1) / 2 - 0.3, ccz, col)
						if top_col then add_face(B, ccx, y1 - 1, ccz, top_col) end
					end
					loft(o0, 0.02, o0, 1.05, body)                                                     -- lower band
					loft(o0, 1.05, o0, BH0, upper)                                                     -- upper walls
					loft(o0, BH0, outline(0.42), BH1, lighten(upper, 4), lighten(upper, 10))           -- bevel + top
					for _, sx in ipairs({ -0.82, 0.82 }) do                                             -- raised pads
						local pad = rrect(0.42, 0.34, 0.08, 2)
						local pp = {}
						for i, p2 in ipairs(pad) do pp[i] = { p2[1] + sx, p2[2] + zi + 0.95 } end
						loft(pp, BH1, pp, BH1 + 0.07, lighten(upper, 6), lighten(upper, 14))
					end
					-- CAUTION stickers on both long sides (the one facing away is culled)
					for _, sgn in ipairs({ 1, -1 }) do
						local sx = sgn * (BW / 2 + 0.012)
						local function Wp(u, y) local wx, wz = rot_y(sx, u, a) return { wx, y, wz } end
						local rx, rz = rot_y(0, bz, a)
						face_bias = 0.6
						decal_any_view = true
						local u0, u1 = zi + 0.6, zi + 0.6 + 2.5              -- 2:1 like the image
						if sx > 0 then u0, u1 = u1, u0 end                   -- read left to right from outside
						add_face({ Wp(u0, 1.6), Wp(u1, 1.6), Wp(u1, 0.35), Wp(u0, 0.35) }, rx, 0.6, rz,
							{ 200, 180, 90 }, TEX.spinner)
						face_bias = 0
						decal_any_view = false
					end
					-- tower and cup (at the gyro's station)
					-- tower (photos): a collar on the box, a wide lower cylinder, a step ring, a narrower upper
					-- cylinder (the sleeve the gyro's leg drops into) and the black cup, just under the gyro's disc
					local TS = GYRO_SEAT + SPIN_UP - 0.12                   -- tower top: just under its gyro's disc
					local Y1 = BH1 + 0.2                                    -- top of the collar
					local Y2 = Y1 + (TS - 0.45 - Y1) * 0.55                 -- the step
					emit_prism(px, BH1 + 0.1, pz, 0.86, 0.2, 18, lighten(post, -4), 0, 0, lighten(post, 4))      -- collar
					emit_prism(px, (Y1 + Y2) / 2, pz, 0.74, Y2 - Y1, 18, lighten(post, 2), 0, 0, lighten(post, 10))  -- lower
					emit_prism(px, Y2 + 0.04, pz, 0.66, 0.08, 18, lighten(tube, -6), 0, 0, lighten(tube, 4))        -- step ring
					emit_prism(px, (Y2 + 0.08 + TS - 0.45) / 2, pz, 0.6, TS - 0.45 - Y2 - 0.08, 18, lighten(tube, 10), 0, 0)  -- upper
					emit_frustum(px, TS - 0.45, pz, 0.6, 0.8, 0.12, 18, { 30, 30, 34 }, 0, 0)            -- cup flare
					cup(px, pz, TS - 0.33, TS - 0.25, TS, 0.8, 0.36, { 34, 34, 38 }, { 18, 18, 20 })
				else                                                      -- gyro holder
					tube_post(px, pz, SEAT - 0.45, 0.45)
					emit_frustum(px, SEAT - 0.45, pz, 0.5, 0.78, 0.12, 18, { 30, 30, 34 }, 0, 0)            -- cup flare
					cup(px, pz, SEAT - 0.33, SEAT - 0.25, SEAT, 0.78, 0.36, { 34, 34, 38 }, { 18, 18, 20 })
				end
			end
		end
		if not fixed then part_keep("buttons") end
		end
		static_end()
		end

		-- vertical extrusion of a convex XZ profile (base, shoulder housing)
		local function emit_yprofile(cx, y0, cz, prof, h, rgb, yaw, top_rgb)
			yaw = yaw or 0
			local bot, top = {}, {}
			for i = 1, #prof do
				local x, z = rot_y(cx + prof[i][1], cz + prof[i][2], yaw)
				bot[i] = { x, y0, z }
				top[i] = { x, y0 + h, z }
			end
			-- orient the faces about the profile's own centre (profiles may sit far from cx, cz)
			local mx, mz = 0, 0
			for i = 1, #prof do mx, mz = mx + bot[i][1], mz + bot[i][3] end
			mx, mz = mx / #prof, mz / #prof
			emit_rings(bot, top, mx, y0 + h / 2, mz, rgb, top, top_rgb or rgb, bot, rgb)
		end

		local band, armcol, hand = COL.band, COL.arm, COL.hand
		local yaw = R.ang * RAD
		local yc = arm_y()                       -- hand height
		-- BODY, measured from the 3D model of the NES unit (Sketchfab "Rob Nintendo"; plugin units:
		-- x across, y up, z forward, 0 = mast centre).
		-- The shoulder housing rides up and down the mast carrying the arms, which stay horizontal (as on
		-- the 3D model): the hands are at the housing's mid-height (0.96 above its bottom).
		local SB = yc - 0.96                     -- housing bottom
		local ST = SB + 2.13                     -- housing top (centre block)
		local HB = 10.83                         -- top of the mast (head flange)
		local function inset(prof, k)
			local cx, cz = 0, 0
			for _, q in ipairs(prof) do cx, cz = cx + q[1], cz + q[2] end
			cx, cz = cx / #prof, cz / #prof
			local out = {}
			for i, q in ipairs(prof) do out[i] = { cx + (q[1] - cx) * k, cz + (q[2] - cz) * k } end
			return out
		end
		local function plate(x0, y0, z0, x1, y1, z1, w0, w1, th, rgb, th1)
			th1 = th1 or th
			local dx, dz = x1 - x0, z1 - z0
			local l = sqrt(dx * dx + dz * dz)
			if l < 1e-6 then return end
			local sx, sz = -dz / l, dx / l                     -- horizontal side vector
			local c = {}
			for i = 0, 7 do
				local e = (i & 1) ~= 0
				local w = e and w1 or w0
				local a = (i & 4) ~= 0 and 1 or -1
				local b = (i & 2) ~= 0 and 1 or -1
				local px, py, pz = e and x1 or x0, e and y1 or y0, e and z1 or z0
				local rx, rz = rot_y(px + sx * w * a, pz + sz * w * a, yaw)
				c[i + 1] = { rx, py + (e and th1 or th) * b, rz }
			end
			local mx, mz = rot_y((x0 + x1) / 2, (z0 + z1) / 2, yaw)
			emit_hexa(c, mx, (y0 + y1) / 2, mz, rgb)
		end

		-- inset a convex polygon (counter-clockwise or clockwise) by d along its edge normals
		local function poly_inset(prof, d)
			local n = #prof
			local area = 0
			for i = 1, n do local p, q = prof[i], prof[i % n + 1] area = area + p[1] * q[2] - q[1] * p[2] end
			local sgn = (area > 0) and 1 or -1
			local lines = {}
			for i = 1, n do
				local p, q = prof[i], prof[i % n + 1]
				local ex, ez = q[1] - p[1], q[2] - p[2]
				local l = sqrt(ex * ex + ez * ez)
				local nx, nz = -ez / l * sgn, ex / l * sgn                -- inward normal
				lines[i] = { p[1] + nx * d, p[2] + nz * d, ex, ez }
			end
			local out = {}
			for i = 1, n do
				local a, b = lines[(i - 2) % n + 1], lines[i]
				local den = a[3] * b[4] - a[4] * b[3]
				if abs(den) < 1e-9 then out[i] = { b[1], b[2] } else
					local t = ((b[1] - a[1]) * b[4] - (b[2] - a[2]) * b[3]) / den
					out[i] = { a[1] + a[3] * t, a[2] + a[4] * t }
				end
			end
			return out
		end
		-- vertical loft between two polygons with the same number of points
		local function emit_yloft(p0, y0, p1, y1, col, top_col)
			local n = #p0
			local a, b = {}, {}
			local cx, cz = 0, 0
			for i = 1, n do
				a[i], b[i] = { p0[i][1], y0, p0[i][2] }, { p1[i][1], y1, p1[i][2] }
				cx, cz = cx + p0[i][1], cz + p0[i][2]
			end
			cx, cz = cx / n, cz / n
			emit_rings(a, b, cx, (y0 + y1) / 2, cz, col, top_col and b, top_col)
		end

		-- BASE, measured from the design patent's orthographic views (US D296,347: front, sides, top,
		-- bottom) and checked against photos of real units. An octagon (front and back edges +-2.35,
		-- widest +-4.76 just behind the middle, 9.1 deep, centred on the mast): dark band 0..0.62, walls
		-- up to 2.0, a small chamfer to the side wings' flat top at 2.25. Down the middle runs a raised
		-- band, as wide as the front and back edges (x +-2.35), up to 2.74: its front end carries the
		-- sticker on a sloped chamfer and the front slot; its rear end carries the vents and the power
		-- switch, and the cable plugs into its rear wall.
		local BAND_H, WALL_H, WING_H, TOP_H = 0.62, 2.0, 2.25, 2.74
		local CBX = 2.35                                       -- raised centre band half width
		local BZF, BZB = 4.55, -4.55                           -- front / back edges
		local band_p = poly_inset(base_p, 0.06)
		if static_begin() then
		do
			local function walls(prof, y0, h, col)
				for i = 1, #prof do
					local p0, p1 = prof[i], prof[i % #prof + 1]
					split_faces = true
					add_face({ { p0[1], y0, p0[2] }, { p1[1], y0, p1[2] }, { p1[1], y0 + h, p1[2] }, { p0[1], y0 + h, p0[2] } },
						0, y0 + h / 2, 0, col)
					split_faces = false
				end
			end
			walls(band_p, 0, BAND_H, band)
			walls(base_p, BAND_H, WALL_H - BAND_H, shell)
		end
		local wing_p = poly_inset(base_p, 0.22)
		-- chamfer, and the wing tops only where they show: outside the raised centre band (one big face
		-- running under the band and the turntable sorts badly against the collar standing over it)
		emit_yloft(base_p, WALL_H, wing_p, WING_H, lighten(shell, 6), nil)
		for _, sg in ipairs({ -1, 1 }) do
			local out = {}                                       -- wing_p clipped to sg * x >= CBX
			for i = 1, #wing_p do
				local p0, p1 = wing_p[i], wing_p[i % #wing_p + 1]
				local in0, in1 = sg * p0[1] >= CBX, sg * p1[1] >= CBX
				if in0 then out[#out + 1] = { p0[1], WING_H, p0[2] } end
				if in0 ~= in1 then
					local t = (sg * CBX - p0[1]) / (p1[1] - p0[1])
					out[#out + 1] = { sg * CBX, WING_H, p0[2] + (p1[2] - p0[2]) * t }
				end
			end
			if #out >= 3 then add_face(out, sg * 3.5, WING_H - 1, 0, lighten(shell, 4)) end
		end
		-- the raised centre band: side walls above the wings, front wall + sloped chamfer, rear wall, top
		local FCH_Z, FCH_Y = 0.58, 2.33                        -- front chamfer: 0.58 deep, from 2.33 up
		do
			local c = shell
			split_faces = true
			-- (each side wall stops where the round side bulge -- radius 3.0, see below -- is moulded into
			-- it, so the bulge and the band are one solid piece with no wall running through the bulge)
			local zc = sqrt(3.0 * 3.0 - CBX * CBX)
			for _, sx in ipairs({ -CBX, CBX }) do
				add_face({ { sx, WING_H, BZB }, { sx, WING_H, -zc }, { sx, TOP_H, -zc }, { sx, TOP_H, BZB } },
					0, WING_H, 0, lighten(c, -3))
				add_face({ { sx, WING_H, zc }, { sx, WING_H, BZF }, { sx, FCH_Y, BZF }, { sx, TOP_H, BZF - FCH_Z }, { sx, TOP_H, zc } },
					0, WING_H, 0, lighten(c, -3))
			end
			add_face({ { -CBX, WALL_H, BZF }, { CBX, WALL_H, BZF }, { CBX, FCH_Y, BZF }, { -CBX, FCH_Y, BZF } }, 0, 1, 0, c)   -- front, above the walls
			add_face({ { -CBX, FCH_Y, BZF }, { CBX, FCH_Y, BZF }, { CBX, TOP_H, BZF - FCH_Z }, { -CBX, TOP_H, BZF - FCH_Z } }, 0, 1, 0, lighten(c, 4))  -- chamfer
			add_face({ { -CBX, WALL_H, BZB }, { CBX, WALL_H, BZB }, { CBX, TOP_H, BZB }, { -CBX, TOP_H, BZB } }, 0, 1, 0, c)   -- rear, above the walls
			split_faces = false
		end
		face_bias = 0.8
		for s = 0, 4 do                                         -- slots (one per accessory station)
			-- an attached accessory's tab fills its slot (and drawing the slot over it would show through)
			if not (STATIONS and STATIONS[s]) then
			local a = (s - 2) * 60 * RAD
			local r = prof_reach(base_p, a)
			emit_box(0, 1.25, r + 0.06, 0.8, 0.55, 0.06, lighten(shell, -8), a)                 -- raised slot pad
			face_bias = 1.2
			emit_box(0, 1.12, r + 0.13, 0.13, 0.38, 0.03, lighten(shell, -75), a)                -- T slot
			emit_box(0, 1.52, r + 0.13, 0.36, 0.08, 0.03, lighten(shell, -75), a)
			face_bias = 0.8
			end
		end
		-- sticker on the sloped chamfer at the front of the centre band
		do
			face_bias = 1.0
			local fc = (machine_kind == "famicom")
			local tex = fc and TEX.sticker_fc or TEX.sticker
			local function S(t, x, lift)                       -- point on the chamfer, t 0 (bottom) .. 1 (top)
				return { x, FCH_Y + (TOP_H - FCH_Y) * t + lift * 0.82, BZF - FCH_Z * t + lift * 0.57 }
			end
			local hw = fc and 2.1 or 1.75
			local t0, t1 = 0.12, 0.88
			if fc then t0, t1 = 0.33, 0.67 end
			add_face({ S(t1, -hw, 0.012), S(t1, hw, 0.012), S(t0, hw, 0.012), S(t0, -hw, 0.012) }, 0, 0, 0, COL.labelbg, tex)
			face_bias = 0.8
		end
		do
			-- rear end of the centre band: vents (+x) and the recessed power switch (-x) in its top,
			-- cable socket low in its rear wall (patent top / rear views; photos)
			local BT = TOP_H
			local function quad(q, rx, ry, rz, c) add_face(q, rx, ry, rz, c) end
			local function top(x0, x1, z0, z1)
				if x1 - x0 < 1e-3 or z1 - z0 < 1e-3 then return end
				quad({ { x0, BT, z0 }, { x1, BT, z0 }, { x1, BT, z1 }, { x0, BT, z1 } }, (x0 + x1) / 2, BT - 1, (z0 + z1) / 2, lighten(shell, 8))
			end
			local function recess(x0, x1, z0, z1, depth, floor_col, open_x1)
				local yf = BT - depth
				local cx, cz = (x0 + x1) / 2, (z0 + z1) / 2
				quad({ { x0, yf, z0 }, { x1, yf, z0 }, { x1, yf, z1 }, { x0, yf, z1 } }, cx, yf - 1, cz, floor_col)
				local wc = lighten(shell, -36)
				quad({ { x0, yf, z0 }, { x1, yf, z0 }, { x1, BT, z0 }, { x0, BT, z0 } }, cx, BT, z0 - 1, wc)
				quad({ { x0, yf, z1 }, { x1, yf, z1 }, { x1, BT, z1 }, { x0, BT, z1 } }, cx, BT, z1 + 1, wc)
				quad({ { x0, yf, z0 }, { x0, yf, z1 }, { x0, BT, z1 }, { x0, BT, z0 } }, x0 - 1, BT, cz, wc)
				if not open_x1 then
					quad({ { x1, yf, z0 }, { x1, yf, z1 }, { x1, BT, z1 }, { x1, BT, z0 } }, x1 + 1, BT, cz, wc)
				end
			end
			local TZ0, TZ1 = BZB, BZF - FCH_Z                  -- the band's flat top
			local RZ0, RZ1 = -4.16, -2.99                       -- rear details' depth range (patent top view)
			local SX0, SX1 = -1.81, -0.57                        -- switch panel
			local VX0, VX1 = 0.52, CBX                           -- vents: they run out to the band's edge
			-- top, built around the openings
			top(-CBX, CBX, TZ0, RZ0)
			-- the flat top in front of / behind the turntable only (none under it, where it would sort
			-- badly against the collar): strips from the turntable's edge out to the band's ends
			do
				local RT, ns = 2.56, 10
				for k = 0, ns - 1 do
					local xa, xb = -CBX + 2 * CBX * k / ns, -CBX + 2 * CBX * (k + 1) / ns
					local za, zb = sqrt(RT * RT - xa * xa), sqrt(RT * RT - xb * xb)
					quad({ { xa, BT, za }, { xb, BT, zb }, { xb, BT, TZ1 }, { xa, BT, TZ1 } }, (xa + xb) / 2, BT - 1, (za + TZ1) / 2, lighten(shell, 8))
					quad({ { xa, BT, RZ1 }, { xb, BT, RZ1 }, { xb, BT, -zb }, { xa, BT, -za } }, (xa + xb) / 2, BT - 1, (RZ1 - za) / 2, lighten(shell, 8))
				end
			end
			top(-CBX, SX0, RZ0, RZ1)
			top(SX1, VX0, RZ0, RZ1)
			top(VX1, CBX, RZ0, RZ1)
			recess(SX0, SX1, RZ0, RZ1, 0.18, { 26, 26, 30 })
			if TEX.switch then
				local yf = BT - 0.18 + 0.004
				decal_any_view = true
				face_bias = 0.3
				add_face({ { SX1 - 0.04, yf, RZ1 - 0.04 }, { SX0 + 0.04, yf, RZ1 - 0.04 }, { SX0 + 0.04, yf, RZ0 + 0.04 }, { SX1 - 0.04, yf, RZ0 + 0.04 } },
					(SX0 + SX1) / 2, yf - 1, (RZ0 + RZ1) / 2, { 26, 26, 30 }, TEX.switch)
				face_bias = 0.8
				decal_any_view = false
			end
			-- vents: nine slats across, 0.13 apart, wrapping round the corner a little way down the
			-- band's side wall (photos)
			local n, VW = 9, 0.035
			local zc = RZ0
			for k = 0, n - 1 do
				local z = RZ0 + 0.06 + k * (RZ1 - RZ0 - 0.12) / (n - 1)
				top(VX0, VX1, zc, z - VW)
				recess(VX0, VX1, z - VW, z + VW, 0.1, { 30, 30, 34 }, true)
				zc = z + VW
				-- down the side wall: a groove from the top edge to 0.32 below it
				decal_any_view = true
				face_bias = 1.6                                   -- the wall is cut into pieces: pull harder
				quad({ { CBX + 0.004, BT, z - VW }, { CBX + 0.004, BT, z + VW }, { CBX + 0.004, BT - 0.32, z + VW }, { CBX + 0.004, BT - 0.32, z - VW } },
					CBX - 1, BT - 0.16, z, { 30, 30, 34 })
				face_bias = 0.8
				decal_any_view = false
			end
			top(VX0, VX1, zc, RZ1)
			-- cable socket: low in the middle of the rear wall
			emit_box(0, 0.5, BZB - 0.05, 0.42, 0.46, 0.05, { 40, 40, 46 }, 0)
		end
		face_bias = 0

		-- ROUND SIDE BULGES moulded into the base (photos): the circle under the turntable (radius 3.0)
		-- sticks out past the raised centre band over the side wings as two rounded areas, flush with the
		-- band's top, with crisp walls down to the wings. The TURNTABLE (a thin disc, radius 2.58, with a
		-- groove ring) sits on the band over a dark seam; the round COLLAR (radius 1.25 .. 1.32, up to
		-- 4.15) carries the mast. Turntable and collar turn with the body.
		do
			local RB, n = 3.0, 12
			local a1 = math.acos(CBX / RB)
			for _, sg in ipairs({ -1, 1 }) do
				local top = {}
				for i = 0, n do
					local t = -a1 + 2 * a1 * i / n
					top[#top + 1] = { sg * RB * cos(t), TOP_H, RB * sin(t) }
				end
				for i = 1, n do
					local p0, p1 = top[i], top[i + 1]
					add_face({ { p0[1], WING_H, p0[3] }, { p1[1], WING_H, p1[3] }, p1, p0 }, 0, 1.0, 0, lighten(shell, -6))
				end
				-- its flat top, in strips across, stopping at the turntable's edge (none under it)
				local zr = RB * sin(a1)
				local RT, ns = 2.56, 12
				for k = 0, ns - 1 do
					local za, zb = -zr + 2 * zr * k / ns, -zr + 2 * zr * (k + 1) / ns
					local function x_in(z) return max(CBX, sqrt(max(0, RT * RT - z * z))) end
					local function x_out(z) return sqrt(max(0, RB * RB - z * z)) end
					local q = { { sg * x_in(za), TOP_H, za }, { sg * x_out(za), TOP_H, za }, { sg * x_out(zb), TOP_H, zb }, { sg * x_in(zb), TOP_H, zb } }
					add_face(q, sg * 2.7, 1.0, (za + zb) / 2, shell)
				end
			end
		end
		end
		static_end()
		local pose = string.format("%.17g|%.17g|%.17g|%s", yaw, yc, claw_gap(), game)
		if not part_reuse("body", pose) then
		local TT_TOP = 2.92
		emit_prism(0, 2.76, 0, 2.54, 0.04, 28, { 40, 40, 44 }, yaw, 0, { 40, 40, 44 })                   -- seam
		emit_prism(0, (2.78 + TT_TOP) / 2, 0, 2.58, TT_TOP - 2.78, 28, COL.table, yaw, 0, lighten(COL.table, 10))
		face_bias = 0.3
		do                                                      -- groove ring on the turntable top
			local n, col = 32, lighten(COL.table, -40)
			for i = 0, n - 1 do
				local a0, a1 = i * 2 * math.pi / n, (i + 1) * 2 * math.pi / n
				local function P(r, a) local x, z = rot_y(r * cos(a), r * sin(a), yaw) return { x, TT_TOP + 0.002, z } end
				add_face({ P(2.22, a0), P(2.29, a0), P(2.29, a1), P(2.22, a1) }, 0, 0, 0, col)
			end
		end
		face_bias = 0
		-- collar (hidden inside the hollow housing when it is lowered over it)
		local col_top = min(4.15, SB)
		if col_top > TT_TOP + 0.04 then
			-- radius 1.32 at the foot, 1.25 at the top: the mast's corners (1.19 from the axis) sit inside it
			emit_frustum(0, TT_TOP, 0, 1.32, 1.25, col_top - TT_TOP, 24, shell, yaw, 0, lighten(shell, 8))
		end

		-- MAST (patent front / side views): a post 1.85 wide and 1.51 deep from the collar to the head.
		-- It turns with the body (the head stays facing the TV). Drawn in two pieces (below / above the
		-- housing) so the painter's sort stays clean.
		local MX, MZ = 0.925, 0.755
		local function mast(y0, y1)
			if y1 - y0 < 0.05 then return end
			emit_box(0, (y0 + y1) / 2, 0, MX, (y1 - y0) / 2, MZ, lighten(shell, -14), yaw)
			-- the front face has a toothed rack running up both edges; the side faces have vertical
			-- grooves. Teeth sit at fixed heights on the mast.
			-- Each detail sorts just in front of the mast face it sits on: its depth-sort bias is the depth
			-- difference between its centre and that face's centre (which moves with the housing's height,
			-- and is far from the lower teeth when the camera looks down), plus a little.
			local ym, hy = (y0 + y1) / 2, (y1 - y0) / 2
			local X = cam._xf
			local function depth(x, y, z)                  -- (x, z on the mast, turned with the body)
				x, z = rot_y(x, z, yaw)
				return -X[4] * (y - X[6]) + X[3] * (-X[2] * x + X[1] * z)
			end
			local dv = decal_any_view
			decal_any_view = true
			for _, gx in ipairs({ -MX - 0.01, MX + 0.01 }) do
				local fd = depth(gx < 0 and -MX or MX, ym, 0)                                 -- the side face's centre
				for _, gz in ipairs({ -0.3, 0.3 }) do
					face_bias = depth(gx, ym, gz) - fd + 0.1
					emit_box(gx, ym, gz, 0.02, hy, 0.035, lighten(shell, -55), yaw)                -- side grooves
				end
			end
			local RIB0, RIB_STEP = 3.2, 0.2
			local fd = depth(0, ym, MZ)                                                       -- the front face's centre
			for k = ceil((y0 + 0.05 - RIB0) / RIB_STEP), floor((y1 - 0.05 - RIB0) / RIB_STEP) do
				for _, tx in ipairs({ -0.72, 0.72 }) do
					face_bias = depth(tx, RIB0 + k * RIB_STEP, MZ) - fd + 0.1
					emit_box(tx, RIB0 + k * RIB_STEP, MZ, 0.14, 0.035, 0.03, lighten(shell, -60), yaw)  -- rack teeth
				end
			end
			face_bias, decal_any_view = 0, dv
		end
		mast(min(4.15, SB), SB)
		mast(ST + 0.1, HB)

		split_faces = true
		-- correction from the design patent's side / front views: the housing reaches 1.82 ahead of the
		-- mast and 2.83 behind it, and is 2.03 tall (the shapes below were measured from a 3D model that
		-- turned out deeper and taller than the real unit)
		do
			local sb, c, s = SB, cos(yaw), sin(yaw)
			face_xf = function(p)
				-- undo the body yaw, rescale, redo it (rot_y by -yaw, then by yaw)
				local x, z = p[1], p[3]
				local lx, lz = x * c - z * s, x * s + z * c
				lz = -0.101 + 0.906 * lz
				return { lx * c + lz * s, sb + (p[2] - sb) * 0.953, -lx * s + lz * c }
			end
		end
		-- SHOULDER HOUSING: a centre block around the mast (x +-2.5, z -3.0..2.0, 2.09 tall, raised panel
		-- on top) and a lobe on each side (to x +-4.64, 1.79 tall) whose front edge sweeps back from the
		-- centre block and whose back curves round. Each lobe has an opening at the front for its arm.
		local LOBE = { { 2.2, 0.8 }, { 2.8, 0.8 }, { 3.0, 0.7 }, { 3.56, 0.5 }, { 3.93, 0.25 }, { 4.25, 0.0 }, { 4.54, -0.25 },
			{ 4.64, -0.5 }, { 4.64, -1.0 }, { 4.54, -1.25 }, { 4.45, -1.5 }, { 4.26, -2.0 }, { 4.07, -2.25 },
			{ 3.84, -2.5 }, { 3.51, -2.75 }, { 2.2, -2.8 } }
		for side = -1, 1, 2 do
			-- one closed shell per lobe: rings at four heights (rounded bottom and top edges), with no
			-- face on the inner side where it joins the centre block -- nothing overlaps, so the
			-- painter's sort cannot show one part through another
			local rings = {}
			-- measured: the lobe reaches x 4.42 at 0.2 above the housing bottom, 4.63 from 0.6 to 1.5,
			-- 4.44 at 2.0; it runs from 0.15 to 2.07
			-- (ring 3 -> 4 is also the height of the arm opening, 0.42 .. 1.8)
			local hs = { { SB + 0.15, 0.91 }, { SB + 0.28, 0.96 }, { SB + 0.42, 1.0 }, { SB + 1.8, 1.0 },
				{ SB + 1.95, 0.96 }, { SB + 2.07, 0.91 } }
			for r, hk in ipairs(hs) do
				local ring = {}
				for i, q in ipairs(LOBE) do
					local x = 2.5 + (q[1] - 2.5) * hk[2]
					local z = -1.1 + (q[2] + 1.1) * hk[2]
					local wx, wz = rot_y(side * x, z, yaw)
					ring[i] = { wx, hk[1], wz }
				end
				rings[r] = ring
			end
			local cx, cz = rot_y(side * 3.6, -1.1, yaw)
			local cy = SB + 1.06
			for r = 1, #rings - 1 do
				local a, b = rings[r], rings[r + 1]
				for i = 1, #LOBE - 1 do                        -- skip the closing (inner) edge
					-- the arm opening: no wall across the front segments 2..6 between rings 3 and 4
					if not (r == 3 and i >= 2 and i <= 6) then
						add_face({ a[i], a[i + 1], b[i + 1], b[i] }, cx, cy, cz, shell)
					end
				end
			end
			add_face(rings[#rings], cx, cy, cz, lighten(shell, 6))
			add_face(rings[1], cx, cy, cz, lighten(shell, -6))
			-- arm opening: a dark recess 0.7 deep behind the front segments 2..6; the arm comes out of it
			do
				local dark, dark2 = lighten(shell, -110), lighten(shell, -95)
				local y0, y1 = SB + 0.42, SB + 1.8
				local fr, bk = {}, {}
				for i = 2, 7 do
					local q = LOBE[i]
					local dx, dz = 3.6 - q[1], -1.1 - q[2]
					local dl = sqrt(dx * dx + dz * dz)
					fr[#fr + 1] = { q[1], q[2] }
					bk[#bk + 1] = { q[1] + dx / dl * 0.7, q[2] + dz / dl * 0.7 }
				end
				local function W(p, y) local x, z = rot_y(side * p[1], p[2], yaw) return { x, y, z } end
				local function ref(x, y, z) local rx, rz = rot_y(side * x, z, yaw) return rx, y, rz end
				local rx, ry, rz
				for i = 1, #fr - 1 do
					-- back wall (seen from the front), ceiling (seen from below), floor (seen from above)
					local bx, bz = (bk[i][1] + bk[i + 1][1]) / 2, (bk[i][2] + bk[i + 1][2]) / 2
					rx, ry, rz = ref(bx + (3.6 - bx) * 2, (y0 + y1) / 2, bz + (-1.1 - bz) * 2)
					add_face({ W(bk[i], y0), W(bk[i + 1], y0), W(bk[i + 1], y1), W(bk[i], y1) }, rx, ry, rz, dark)
					rx, ry, rz = ref(bx, y1 + 1, bz)
					add_face({ W(fr[i], y1), W(fr[i + 1], y1), W(bk[i + 1], y1), W(bk[i], y1) }, rx, ry, rz, dark2)
					rx, ry, rz = ref(bx, y0 - 1, bz)
					add_face({ W(fr[i], y0), W(fr[i + 1], y0), W(bk[i + 1], y0), W(bk[i], y0) }, rx, ry, rz, dark2)
				end
				-- end walls
				local n = #fr
				rx, ry, rz = ref(fr[1][1] - 1, (y0 + y1) / 2, fr[1][2])
				add_face({ W(fr[1], y0), W(bk[1], y0), W(bk[1], y1), W(fr[1], y1) }, rx, ry, rz, dark2)
				rx, ry, rz = ref(fr[n][1] + 1, (y0 + y1) / 2, fr[n][2] - 0.5)
				add_face({ W(fr[n], y0), W(bk[n], y0), W(bk[n], y1), W(fr[n], y1) }, rx, ry, rz, dark2)
			end
			face_bias = 0
		end
		-- centre block ("chest"), measured from the 3D model as horizontal sections. h = height above the
		-- housing bottom, w = half width (the ends bulge: 2.30 at the bottom, 2.64 at mid-height, 2.22 at
		-- the top), f = front (flat 2.12 ahead of the mast, the top front edge rolling back to 1.55),
		-- k = back (3.0 behind the mast). Front corners are rounded 0.4 in plan, back corners 0.25.
		-- A seam runs round it at h 1.06..1.1, and an inset panel (x +-0.91) sits on the rolled top front.
		do
			local SEC = {
				{ 0.0, 2.30, 2.04, -2.95 }, { 0.1, 2.44, 2.11, -3.0 }, { 0.3, 2.55, 2.12, -3.01 },
				{ 0.6, 2.62, 2.12, -3.01 }, { 1.06, 2.64, 2.12, -3.01 }, { 1.1, 2.64, 2.12, -3.01 },
				{ 1.25, 2.63, 2.09, -3.01 }, { 1.35, 2.62, 2.065, -3.0 }, { 1.35, 2.62, 2.065, -3.0, 0.1 },
				{ 1.4, 2.61, 2.05, -3.0, 0.1 }, { 1.6, 2.57, 2.0, -3.0, 0.1 },
				{ 1.8, 2.5, 1.92, -2.99, 0.1 }, { 1.95, 2.42, 1.82, -2.97, 0.1 }, { 2.05, 2.33, 1.73, -2.94, 0.1 },
				{ 2.13, 2.22, 1.55, -2.88, 0.1 },
			}
			local IW = 0.91                     -- half width of the indented centre panel (5th value = its depth)
			local RF, RB, SEG = 0.4, 0.25, 3
			local REC = {}                      -- vertex indices of the indent's back face (same in every ring)
			local function ring(w, f, k, y, d)
				local pts = {}
				local function add(x, z) local wx, wz = rot_y(x, z, yaw) pts[#pts + 1] = { wx, y, wz } end
				local corners = { { w - RF, f - RF, 0, RF }, { -(w - RF), f - RF, 90, RF },
					{ -(w - RB), k + RB, 180, RB }, { w - RB, k + RB, 270, RB } }
				for ci, c in ipairs(corners) do
					for i = 0, SEG do
						local an = (c[3] + i * 90 / SEG) * RAD
						-- plan: x across, z forward; angle 0 = +x, 90 = +z
						add(c[1] + c[4] * cos(an), c[2] + c[4] * sin(an))
					end
					if ci == 1 then                  -- the front edge, with the indent in its middle
						add(IW, f) add(IW, f - d)
						REC[1] = #pts
						add(-IW, f - d) add(-IW, f)
					end
				end
				return pts
			end
			local rings = {}
			for i, q in ipairs(SEC) do rings[i] = ring(q[2], q[3], q[4], SB + q[1], q[5] or 0) end
			local mx, mz = rot_y(0, -0.45, yaw)
			local my = SB + 1.06
			local seam = lighten(shell, -38)
			for r = 1, #rings - 1 do
				local A, B = rings[r], rings[r + 1]
				local col = (SEC[r][1] == 1.06) and seam or shell
				for i = 1, #A do
					local j = i % #A + 1
					local c2 = col
					if i == REC[1] and (SEC[r][5] or 0) > 0 then c2 = lighten(shell, -9) end   -- indent back face
					-- the indent's side walls face into the indent (toward the middle), not away from it
					local rx, rz = mx, mz
					if i == REC[1] - 1 then rx, rz = rot_y(IW * 3, 1.0, yaw)
					elseif i == REC[1] + 1 then rx, rz = rot_y(-IW * 3, 1.0, yaw) end
					add_face({ A[i], A[j], B[j], B[i] }, rx, my, rz, c2)
				end
			end
			add_face(rings[#rings], mx, my, mz, lighten(shell, 6))                                     -- top
			add_face(rings[1], mx, my, mz, lighten(shell, -6))                                         -- bottom
			emit_box(0, ST + 0.05, -1.05, 0.9, 0.05, 1.87, lighten(shell, 4), yaw)                    -- raised top panel
		end

		face_xf = nil                                      -- the cable ends are placed directly
		-- COILED CABLE: plugged into the centre of the base's back wall (fixed) at one end and into the
		-- centre of the shoulder housing's back at the other, so it stretches and swings as the housing turns.
		do
			emit_box(0, SB + 0.9, -2.83, 0.24, 0.2, 0.08, { 52, 52, 58 }, yaw)      -- cable boot on the housing
			local x0, z0 = rot_y(0, -2.86, yaw)
			local y0 = SB + 0.9
			local x1, y1, z1 = 0, 0.9, -4.64                     -- into the socket low in the rear wall (fixed)
			emit_box(0, 0.9, -4.62, 0.16, 0.06, 0.05, { 30, 30, 34 }, 0)            -- plug
			-- the path: a curve leaving the housing outward and down, coming down outside the rear wall
			-- into the socket. It bows out and droops (floppy), and is then kept clear of the model: above
			-- whatever part of the base it is over, outside the mast and collar, above the floor.
			local rl = sqrt(x0 * x0 + z0 * z0)
			local bx, bz = x0 / rl, z0 / rl                        -- straight out from the mast at the boot
			local ex, ey, ez = 0, 1.1, BZB - 0.5                   -- just outside the rear wall, above the socket
			local P = {
				{ x0, y0, z0 },
				{ x0 + bx * 1.8, y0 - 0.8, z0 + bz * 1.8 },
				{ ex, ey + 2.4, ez - 1.0 },
				{ ex, ey, ez },
			}
			local function bez(t)
				local u = 1 - t
				local w = { u * u * u, 3 * u * u * t, 3 * u * t * t, t * t * t }
				local q = { 0, 0, 0 }
				for k = 1, 4 do for c = 1, 3 do q[c] = q[c] + w[k] * P[k][c] end end
				return q
			end
			local mxw, mzw = (x0 + ex) / 2, (z0 + ez) / 2
			local ml = sqrt(mxw * mxw + mzw * mzw)
			if ml < 1e-3 then mxw, mzw, ml = 0, -1, 1 end
			local ox, oz = mxw / ml, mzw / ml                    -- bow outward, away from the mast
			-- how far inside the base outline (x, z) is (negative: outside)
			local edges = {}                                          -- base outline edges: start, unit direction
			for k = 1, #base_p do
				local p1, p2 = base_p[k], base_p[k % #base_p + 1]
				local dx, dz = p2[1] - p1[1], p2[2] - p1[2]
				local l = sqrt(dx * dx + dz * dz)
				edges[k] = { p1[1], p1[2], dx / l, dz / l }
			end
			local function inside_base(x, z)
				local d = huge
				for k = 1, #edges do
					local e = edges[k]
					local sd = (x - e[1]) * e[4] - (z - e[2]) * e[3]
					if sd < d then d = sd end
				end
				return d
			end
			local sgn = (inside_base(0, 0) > 0) and 1 or -1           -- outline winding
			local CLR = 0.42                                          -- coil radius + wire
			local function clear(x, y, z)
				local r = sqrt(x * x + z * z)
				if y < SB and r < 1.32 + CLR then                     -- the mast / collar
					local k = (1.32 + CLR) / max(r, 1e-3)
					if r < 1e-3 then x, z = 0, -(1.32 + CLR) else x, z = x * k, z * k end
					r = 1.32 + CLR
				end
				if sgn * inside_base(x, z) > -CLR then                -- over the base: stay above its top
					local top = (r < 2.6 + CLR) and TT_TOP or ((abs(x) < CBX + CLR or r < 3.0 + CLR) and TOP_H or WING_H)
					y = max(y, top + CLR)
				end
				return x, max(y, CLR), z
			end
			local turns, cr = 23, 0.3
			local n = turns * 7
			local path = {}
			for i = 0, n do
				local t = i / n
				local q = bez(t)
				local bow = sin(t * math.pi)
				local cx, cy, cz = clear(q[1] + ox * 1.2 * bow, q[2] - 0.6 * bow, q[3] + oz * 1.2 * bow)
				local a = (t ^ 0.75) * turns * 2 * math.pi
				path[i] = { cx + cr * cos(a), cy, cz + cr * sin(a) }
			end
			-- one continuous square tube (0.18 thick) along the path: a ring of four corners at each
			-- point, square to the path there, with two sides level (as the separate bars had)
			local WR, wire = 0.09, { 30, 30, 34 }
			local ring_prev
			for i = 0, n do
				local pa, pb = path[i > 0 and i - 1 or 0], path[i < n and i + 1 or n]
				local dx, dy, dz = pb[1] - pa[1], pb[2] - pa[2], pb[3] - pa[3]
				local dl = sqrt(dx * dx + dy * dy + dz * dz)
				dx, dy, dz = dx / dl, dy / dl, dz / dl
				local ux, uy, uz = 0, 1, 0
				if abs(dy) > 0.95 then ux, uy, uz = 1, 0, 0 end
				local sx, sy, sz = dy * uz - dz * uy, dz * ux - dx * uz, dx * uy - dy * ux
				local sl = sqrt(sx * sx + sy * sy + sz * sz)
				sx, sy, sz = sx / sl, sy / sl, sz / sl
				local vx, vy, vz = sy * dz - sz * dy, sz * dx - sx * dz, sx * dy - sy * dx
				local p = path[i]
				local ring = {}
				for k, ab in ipairs({ { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } }) do
					local a, b = ab[1] * WR, ab[2] * WR
					ring[k] = { p[1] + sx * a + vx * b, p[2] + sy * a + vy * b, p[3] + sz * a + vz * b }
				end
				if ring_prev then
					local q = path[i - 1]
					emit_rings(ring_prev, ring, (q[1] + p[1]) / 2, (q[2] + p[2]) / 2, (q[3] + p[3]) / 2, wire,
						i == 1 and ring_prev or nil, wire, i == n and ring or nil, wire)
				end
				ring_prev = ring
			end
			emit_bar(ex, ey, ez, 0, 0.9, BZB - 0.08, 0.09, 0.09, { 30, 30, 34 }, 0)   -- straight into the socket
		end

		face_xf = nil
		-- ARMS (measured from the 3D model, right arm as posed with the claws open; mirrored for the left):
		-- a long bar from the pivot in the lobe to a ring on the hand -- wide near the shoulder, narrow
		-- in the middle -- made of an upper and a lower plate. The hand is a smoked-black block running
		-- front-to-back. Gyromite clips a grey claw on it: two stacked V-notched plates pointing inward,
		-- whose notches close round the gyro spindle. Closing swings the whole arm about the pivot; the
		-- arm pitches down from the pivot (y SB + 0.96) to the hand height.
		local gyro_claws = (game ~= "stackup")                       -- Stack-Up hands carry square foam pads
		local gap = claw_gap()
		-- ARM (photos of real units, patent top / side views), right arm, posed with the grippers open.
		-- Plan: a wide shoulder piece runs out of the lobe's opening, outward to an elbow outside the
		-- housing; the forearm then runs forward to the hand. Side: the grey upper bar ends over the hand
		-- in a rounded end with a hole; the black lower bar runs the full length beneath it. The hand is a
		-- black bar lying across (x), with the gripper (claw / foam grip) on its inner end.
		local AX, AZ = ARM_APEX_X, ARM_APEX_Z
		local HX0, HX1 = AX + 1.6, AX + 3.4                           -- hand bar, inner / outer end
		local ROOT = { 3.95, 0.0 }
		local ELB = { 5.5, 2.3 }
		local FE = { HX1 - 0.65, AZ - 0.1 }                           -- forearm end (over the hand bar's outer half)
		for side = -1, 1, 2 do
			-- swing angle that puts the grip point at x = gap / 2
			local d = arm_swing(gap / 2)
			local cd, sd = cos(d), sin(d)
			local function T(x, z)                                    -- posed (right arm) point -> this arm, now
				local dx, dz = x - ARM_PIV_X, z - ARM_PIV_Z
				return side * (ARM_PIV_X + dx * cd + dz * sd), ARM_PIV_Z - dx * sd + dz * cd
			end
			-- the arm is rigid from the shoulder pivot to the hand pivot (the rounded end's hole); the
			-- hand hangs on that pivot without turning, so both hands stay parallel
			local hpx, hpz = T(AX + HP_DX, AZ + HP_DZ)
			local function TH(x, z) return hpx + side * (x - AX - HP_DX), hpz + (z - AZ - HP_DZ) end
			local lower = { 40, 40, 44 }                              -- the black lower bar
			local function bar(p0, p1, w0, w1, y0, y1, col, y1b)       -- a bar along the arm (y: from the hand height)
				local ax, az = T(p0[1], p0[2])
				local bx2, bz2 = T(p1[1], p1[2])
				y1b = y1b or y1
				plate(ax, yc + (y0 + y1) / 2, az, bx2, yc + (y0 + y1b) / 2, bz2, w0, w1, (y1 - y0) / 2, col, (y1b - y0) / 2)
			end
			-- Three stacked pieces (side photos of real units): a grey TOP piece -- tall where it leaves
			-- the shoulder, then a thinner bar ending in a rounded end on top of the hand; a thin BLACK
			-- MIDDLE bar, the pivot linkage that carries the hand, running the full length into the hand
			-- block; and a grey BOTTOM piece running under the hand, also thicker near the shoulder.
			-- Small spacer posts join the grey pieces at the elbow and near the hand.
			local grey_b = lighten(armcol, -14)
			-- top piece
			bar(ROOT, ELB, 0.5, 0.7, 0.13, 0.75, armcol, 0.45)
			bar(ELB, FE, 0.42, 0.32, 0.13, 0.41, armcol)
			local RS0 = { ELB[1] + (FE[1] - ELB[1]) * 0.55, ELB[2] + (FE[2] - ELB[2]) * 0.55 }
			local RS1 = { ELB[1] + (FE[1] - ELB[1]) * 0.75, ELB[2] + (FE[2] - ELB[2]) * 0.75 }
			bar(RS0, RS1, 0.3, 0.3, 0.41, 0.48, lighten(armcol, 6))                    -- raised section
			-- black middle bar (pivot linkage) into the hand block
			bar(ROOT, ELB, 0.34, 0.4, -0.12, 0.07, lower)
			bar(ELB, { (HX0 + HX1) / 2, AZ }, 0.3, 0.26, -0.12, 0.07, lower)
			-- bottom piece
			bar(ROOT, ELB, 0.5, 0.7, -0.6, -0.18, grey_b, -0.43)
			bar(ELB, FE, 0.4, 0.3, -0.43, -0.18, grey_b)
			-- spacer posts between the grey pieces
			for _, f in ipairs({ 0.12, 0.88 }) do
				local px2, pz2 = T(ELB[1] + (FE[1] - ELB[1]) * f, ELB[2] + (FE[2] - ELB[2]) * f)
				emit_prism(px2, yc - 0.025, pz2, 0.1, 0.31, 8, lighten(armcol, -24), yaw, 0)
			end
			-- elbow: the grey pieces widen into a broad joint (top and bottom), the black bar between
			local exw, ezw = T(ELB[1], ELB[2])
			emit_prism(exw, yc + 0.29, ezw, 0.7, 0.32, 10, armcol, yaw, 0, lighten(armcol, 10))
			emit_prism(exw, yc - 0.305, ezw, 0.68, 0.25, 10, grey_b, yaw, 0)
			-- hand block (black, across), the top piece's rounded end on it, studs
			-- hand bar (overhead photos): a tapered plate -- full depth at the gripper, its front edge
			-- straight, its back edge sloping in toward the outer end, the outer corners rounded
			do
				local pts = { { HX0, AZ - 0.5 }, { HX0, AZ + 0.5 } }
				for k = 0, 3 do                                     -- front outer corner
					local t = k * (math.pi / 2) / 3
					pts[#pts + 1] = { HX1 - 0.25 + 0.25 * sin(t), AZ + 0.25 + 0.25 * cos(t) }
				end
				for k = 0, 3 do                                     -- back outer corner
					local t = k * (math.pi / 2) / 3
					pts[#pts + 1] = { HX1 - 0.18 + 0.18 * cos(t), AZ - 0.15 - 0.18 * sin(t) }
				end
				local q = {}
				for i, p2 in ipairs(pts) do local x, z = TH(p2[1], p2[2]) q[i] = { x, z } end
				emit_yprofile(0, yc - 0.44, 0, q, 0.56, hand, yaw, lighten(hand, 14))
			end
			local ex, ez = T(FE[1], FE[2])
			emit_prism(ex, yc + 0.27, ez, 0.34, 0.28, 12, armcol, yaw, 0, lighten(armcol, 14))         -- rounded end
			face_bias = 0.5
			emit_prism(ex, yc + 0.415, ez, 0.15, 0.01, 10, lighten(armcol, -60), yaw, 0, lighten(armcol, -60))  -- hole
			for _, st in ipairs({ { HX0 + 0.35, AZ }, { HX1 - 0.25, AZ + 0.25 } }) do
				local sx, sz = TH(st[1], st[2])
				emit_prism(sx, yc + 0.16, sz, 0.11, 0.08, 8, hand, yaw, 0, lighten(hand, 34))           -- studs
			end
			face_bias = 0
			if gyro_claws then
				-- Gyromite claw (real photos: front, top, three-quarter, underside). Hand frame: u outward
				-- along the hand bar from the notch apex region, v front/back, y from the arm height.
				--   body  : a solid grey block over the black bar's inner end, flat top, its outer bottom
				--           corner rounded, hanging a little below the bar
				--   rib   : a raised strip across the top and down both faces, a third of the way along
				--   prongs: front and back, round a V-notch that closes on the spindle (seen from above a
				--           "cat's head"); seen from the front each prong forks -- a thick upper fin and a
				--           thinner, shorter lower blade with an open slot between them
				local cc = COL.claw
				local function V(u, y, v)
					local wx, wz = TH(AX + u, AZ + v)
					local rx, rz = rot_y(wx, wz, yaw)
					return { rx, yc + y, rz }
				end
				-- solid from a convex (u, v) plan, between heights y0 .. y1 (numbers, or functions of u)
				local function ext_y(plan, y0, y1, col)
					local f0 = type(y0) == "function" and y0 or function() return y0 end
					local f1 = type(y1) == "function" and y1 or function() return y1 end
					local A, B, cu, cv, cy = {}, {}, 0, 0, 0
					for i, q in ipairs(plan) do
						A[i], B[i] = V(q[1], f0(q[1]), q[2]), V(q[1], f1(q[1]), q[2])
						cu, cv, cy = cu + q[1], cv + q[2], cy + (f0(q[1]) + f1(q[1])) / 2
					end
					local m = V(cu / #plan, cy / #plan, cv / #plan)
					emit_rings(A, B, m[1], m[2], m[3], col, A, lighten(col, -16), B, lighten(col, 12))
				end
				local D = 0.75                                          -- half depth (front / back)
				local U0, U1 = 0.45, (HX0 - AX) + 0.3                   -- body: inner face .. over the bar's end
				local YB, YT = -0.52, 0.28
				-- body, outer bottom corner rounded
				local prof = { { U0, YB }, { U1 - 0.3, YB } }
				for k = 1, 3 do
					local t = k * math.pi / 8
					prof[#prof + 1] = { U1 - 0.3 + 0.3 * sin(t), YB + 0.3 - 0.3 * cos(t) }
				end
				prof[#prof + 1] = { U1, YB + 0.3 }
				prof[#prof + 1] = { U1, YT }
				prof[#prof + 1] = { U0, YT }
				-- the outer bottom corner is rounded
				-- seen from above the claw is a triangle (overhead photos): the front edge straight, the back
				-- edge sloping from the ear toward the front at the hand bar
				local function vb(u) return -D + 0.6 * clamp((u - U0) / (U1 - U0), 0, 1) end   -- back edge
				local function ybot(u)                                  -- bottom, rounded at the outer end
					local c0 = U1 - 0.3
					if u <= c0 then return YB end
					local du = min(u - c0, 0.3)
					return YB + 0.3 - sqrt(max(0, 0.09 - du * du))
				end
				do
					local us = { U0, U1 - 0.3, U1 - 0.2, U1 - 0.1, U1 - 0.03, U1 }
					local plan = { { U0, vb(U0) } }                        -- (starts at a corner: no three
					for _, u in ipairs(us) do plan[#plan + 1] = { u, D } end      -- in line at the start)
					for k = #us, 2, -1 do plan[#plan + 1] = { us[k], vb(us[k]) } end
					ext_y(plan, ybot, YT, cc)
				end
				-- rib across (follows the taper)
				ext_y({ { 0.78, D + 0.04 }, { 0.96, D + 0.04 }, { 0.96, vb(0.96) - 0.04 }, { 0.78, vb(0.78) - 0.04 } },
					YB - 0.03, YT + 0.05, lighten(cc, 6))
				-- prongs round the V-notch: pointed "ears" (apex at u 0.2, tips at u -0.28, v +-0.6), their
				-- outer edges sloping in from the body's corners
				for _, sv in ipairs({ -1, 1 }) do
					-- seen from above (overhead photos) the claw is one triangle: the FRONT prong runs on from
					-- the straight front edge to a long point far inward; the BACK prong is a short ear
					-- angled backward past the body's back edge; the V-notch lies between them
					local function pl(tip_u, ear_u)
						if sv > 0 then
							return { { tip_u, D }, { U0 + 0.02, D }, { U0 + 0.02, 0 }, { 0.2, 0 } }
						end
						return { { 0.2, 0 }, { U0 + 0.02, 0 }, { U0 + 0.02, -D }, { ear_u or 0.05, -D - 0.2 } }
					end
					if side > 0 then
						-- FORK claw: upper fin angled up toward its tip, thin lower blade, open slot between
						-- (layers alternate with the tongue claw's, top to bottom: V plate / fin / tongue / blade /
						-- V plate, so the empty claws can close right up)
						ext_y(pl(-0.45), -0.04, 0.16, cc)
						ext_y(pl(-0.3, 0.15), -0.43, -0.37, lighten(cc, -10))
					else
						-- TONGUE claw: one blade at the fork's slot height, reaching past the centre into the
						-- other claw's slot when closed -- the two interlock round the spindle
						ext_y(pl(-0.4, 0.1), -0.35, -0.07, lighten(cc, -4))
					end
				end
				if side < 0 then
					-- above and below the tongue (photos): short V-shaped plates -- the same pointed, V-notched
					-- prong pair, thin and stopping just short of the spindle -- that hold the gyro
					for _, sv in ipairs({ -1, 1 }) do
						local q = (sv > 0) and { { -0.2, D }, { U0 + 0.02, D }, { U0 + 0.02, 0 }, { 0.2, 0 } }
							or { { 0.2, 0 }, { U0 + 0.02, 0 }, { U0 + 0.02, -D }, { 0.2, -D - 0.12 } }
						ext_y(q, 0.18, YT, lighten(cc, 4))
						ext_y(q, YB, -0.45, lighten(cc, -8))
					end
				end
			else
				-- Stack-Up block hands (from photos): a square foam pad clipped under the hand, its flat
				-- inner face closing on the block (dark grey on the NES set, red on the Famicom one)
				local foam = (machine_kind == "famicom") and { 176, 44, 44 } or { 82, 82, 88 }   -- holder colour
				local function poly(pts)
					local out = {}
					for i, q in ipairs(pts) do local x, z = TH(q[1], q[2]) out[i] = { x, z } end
					return out
				end
				local px0 = AX                                             -- inner face at the grip line
				-- holder: a box, rectangular seen from the front; a foam block is set into the inner face
				do
					local xi, xo, z0, z1 = px0 + 0.22, HX0, AZ - 0.82, AZ + 0.82
					local yb, yti, yto = yc - 0.45, yc + 0.35, yc + 0.35
					local function V(x, y, z) local wx, wz = TH(x, z) local rx, rz = rot_y(wx, wz, yaw) return { rx, y, rz } end
					-- triangular from above (photos): the front edge straight, the back edge sloping in from the
					-- inner face toward the front at the outer end
					local zo0, zo1 = AZ - 0.4, z1
					local c = {
						V(xi, yb, z0), V(xo, yb, zo0), V(xo, yto, zo0), V(xi, yti, z0),
						V(xi, yb, z1), V(xo, yb, zo1), V(xo, yto, zo1), V(xi, yti, z1),
					}
					local mxw, mzw = TH((xi + xo) / 2, (z0 + z1) / 2)
					local mx2, mz2 = rot_y(mxw, mzw, yaw)
					local my2 = yc - 0.1
					for _, f in ipairs({ { 1, 2, 3, 4 }, { 5, 6, 7, 8 }, { 1, 2, 6, 5 }, { 2, 3, 7, 6 }, { 3, 4, 8, 7 }, { 4, 1, 5, 8 } }) do
						add_face({ c[f[1]], c[f[2]], c[f[3]], c[f[4]] }, mx2, my2, mz2, (f[1] == 3) and lighten(foam, 16) or foam)
					end
				end
				local sponge = { 46, 46, 50 }
				emit_yprofile(0, yc - 0.4, 0, poly({ { px0, AZ - 0.68 }, { px0 + 0.22, AZ - 0.68 }, { px0 + 0.22, AZ + 0.68 }, { px0, AZ + 0.68 } }),
					0.62, sponge, yaw, lighten(sponge, 10))                                        -- foam block
			end
		end
		part_keep("body")
		end
		split_faces, face_xf, face_bias = false, nil, 0
		-- HEAD, measured from a 3D model of the NES unit (scaled by its mast width). Fixed: it always faces
		-- the TV. Coordinates: x across, y up, z forward (0 = mast centre).
		--   body : 4.76 wide, 2.56 tall, front face 2.39 ahead of the mast, back 1.96 behind; front top edge
		--          strongly rounded, bottom edge slightly
		--   tail : full width, 1.26 tall (a little above centre), reaching 3.02 behind the mast
		--   visor: smoked black box protruding 0.99, tapering from 4.32 x 2.0 to 3.84 x 1.8; two lenses
		--          (radius 0.62, at x = +-1.22) flush with its front and slats between them
		--   ears : elongated octagonal vent panels, 2.8 x 1.26, protruding 0.2, with three rounded slots
		--   top  : raised plate with the LED near its front edge; neck: flange on the mast + bracket
		-- extrude a convex (z, y) side profile across x from x0 to x1
		local function emit_xprofile(x0, x1, prof, col)
			local a0, a1 = {}, {}
			local cy, cz = 0, 0
			for i, q in ipairs(prof) do
				a0[i], a1[i] = { x0, q[2], q[1] }, { x1, q[2], q[1] }
				cz, cy = cz + q[1], cy + q[2]
			end
			cz, cy = cz / #prof, cy / #prof
			emit_rings(a0, a1, (x0 + x1) / 2, cy, cz, col, a0, col, a1, col)
		end
		-- loft between two (x, y) profiles at z0 and z1 (same point count): the tapering visor
		local function emit_loft(cy, z0, p0, z1, p1, col, cap)
			local a0, a1 = {}, {}
			for i = 1, #p0 do a0[i] = { p0[i][1], cy + p0[i][2], z0 } a1[i] = { p1[i][1], cy + p1[i][2], z1 } end
			emit_rings(a0, a1, 0, cy, (z0 + z1) / 2, col, a1, cap or col, a0, col)
		end

		local HW = 2.37                                   -- half width (measured +-0.137 model units)
		local H0 = HB + 0.59                              -- head bottom (above the neck bracket)
		-- Side profile measured from the 3D model (model units: y = depth, front = -y; z = height).
		-- Each entry: { depth y, top z, bottom z }; the head bottom is at z = -0.276.
		local PROF = {
			{ -0.070, -0.206, -0.249 }, { -0.066, -0.196, -0.270 }, { -0.060, -0.185, -0.276 }, { -0.050, -0.168, -0.276 },
			{ -0.040, -0.151, -0.276 }, { -0.030, -0.140, -0.276 }, { -0.010, -0.138, -0.276 }, {  0.030, -0.134, -0.276 },
			{  0.070, -0.132, -0.276 }, {  0.110, -0.134, -0.276 }, {  0.140, -0.136, -0.274 }, {  0.160, -0.138, -0.269 },
			{  0.172, -0.140, -0.262 }, {  0.180, -0.144, -0.251 }, {  0.190, -0.153, -0.248 }, {  0.205, -0.165, -0.244 },
			{  0.220, -0.168, -0.242 }, {  0.238, -0.169, -0.237 }, {  0.246, -0.180, -0.232 },
		}
		local function mz(y) return (0.067 - y) * 17.3 end            -- model depth -> our z (0 = mast centre)
		local function my(z) return H0 + (z + 0.276) * 17.3 end       -- model height -> our y
		local H1 = my(-0.132)
		local ZF = mz(-0.070)                                          -- front face (behind the visor)
		-- correction from the design patent's side / front views (and side photos): the head reaches 3.18
		-- ahead of the mast and 2.65 behind, is 4.54 wide and 2.31 tall
		local function head_xf(p) return { p[1] * 0.96, H0 + (p[2] - H0) * 0.93, 0.101 + 0.911 * p[3] } end
		if static_begin() then
		face_xf = head_xf
		-- shell: consecutive profile slices joined by quads; each slice is an octagon (chamfered edges)
		do
			local BEV = 0.22
			local rings = {}
			for i, q in ipairs(PROF) do
				local z, top, bot = mz(q[1]), my(q[2]), my(q[3])
				local bv = min(BEV, (top - bot) / 3)
				rings[i] = { z = z, top = top, bot = bot, pts = {
					{ HW, top - bv }, { HW - bv, top }, { -HW + bv, top }, { -HW, top - bv },
					{ -HW, bot + bv }, { -HW + bv, bot }, { HW - bv, bot }, { HW, bot + bv } } }
			end
			for i = 1, #rings - 1 do
				local r0, r1 = rings[i], rings[i + 1]
				local cz, cy = (r0.z + r1.z) / 2, (r0.top + r0.bot + r1.top + r1.bot) / 4
				for k = 1, 8 do
					local k2 = k % 8 + 1
					local a0, b0, a1, b1 = r0.pts[k], r0.pts[k2], r1.pts[k], r1.pts[k2]
					add_face({ { a0[1], a0[2], r0.z }, { b0[1], b0[2], r0.z }, { b1[1], b1[2], r1.z }, { a1[1], a1[2], r1.z } },
						0, cy, cz, shell)
				end
			end
			for _, r in ipairs({ rings[1], rings[#rings] }) do
				local cap = {}
				for k, q in ipairs(r.pts) do cap[k] = { q[1], q[2], r.z } end
				add_face(cap, 0, (r.top + r.bot) / 2, r.z + ((r == rings[1]) and -1 or 1), lighten(shell, -6))
			end
		end
		-- neck: a round flange plate on top of the mast (wide enough to cover the mast's corners however
		-- the mast is turned), bracket up into the head (on the mast: not rescaled)
		face_xf = nil
		emit_prism(0, HB + 0.05, 0, 1.25, 0.1, 32, lighten(shell, -22), 0, 0, lighten(shell, -22))
		emit_box(0, HB + 0.34, -0.05, 0.7, 0.25, 0.65, lighten(shell, -34), 0)
		face_xf = head_xf
		-- ears (measured from the model): a raised vent panel on each side, flat along the bottom, with
		-- straight sides and chamfered upper corners leading to a narrower flat top; three rounded slots
		local EB = my(-0.240)                                -- bottom edge
		local ear = { { mz(-0.016), EB }, { mz(-0.016), my(-0.200) }, { mz(0.012), my(-0.168) },
			{ mz(0.122), my(-0.168) }, { mz(0.150), my(-0.200) }, { mz(0.150), EB } }
		-- depths (photos): a low floor, one raised outline round its edge, and three raised bars inside
		local ecz, ecy = (mz(-0.016) + mz(0.150)) / 2, (EB + my(-0.168)) / 2
		local inner = {}
		for i, q in ipairs(ear) do inner[i] = { ecz + (q[1] - ecz) * 0.84, ecy + (q[2] - ecy) * 0.8 } end
		local FL, RIM, BAR = 0.2, 0.32, 0.29                 -- floor, outline and bar heights off the head side (front photos: ~0.3)
		for side = -1, 1, 2 do
			emit_xprofile(side * HW, side * (HW + FL), ear, lighten(shell, -8))                 -- floor
			local xo, xi = side * HW, side * (HW + RIM)
			local xm = side * (HW + RIM / 2)
			for i = 1, #ear do
				local j = i % #ear + 1
				local o1, o2, i1, i2 = ear[i], ear[j], inner[i], inner[j]
				local mzo, myo = (o1[1] + o2[1]) / 2, (o1[2] + o2[2]) / 2
				-- top of the outline
				add_face({ { xi, o1[2], o1[1] }, { xi, o2[2], o2[1] }, { xi, i2[2], i2[1] }, { xi, i1[2], i1[1] } },
					xo, ecy, ecz, lighten(shell, 2))
				-- its outer wall (faces away from the ear's centre)
				add_face({ { xo, o1[2], o1[1] }, { xo, o2[2], o2[1] }, { xi, o2[2], o2[1] }, { xi, o1[2], o1[1] } },
					xm, ecy, ecz, lighten(shell, -14))
				-- its inner wall (faces the centre)
				add_face({ { xo, i1[2], i1[1] }, { xo, i2[2], i2[1] }, { xi, i2[2], i2[1] }, { xi, i1[2], i1[1] } },
					xm, ecy + (myo - ecy) * 3, ecz + (mzo - ecz) * 3, lighten(shell, -20))
			end
		end
		local SY = EB + 0.6                                   -- bar centre height
		for side = -1, 1, 2 do
			for k = -1, 1 do
				local z = ecz + k * 0.62
				emit_box(side * (HW + (FL + BAR) / 2), SY, z, (BAR - FL) / 2, 0.38, 0.1, lighten(shell, 0), 0)          -- bar
				for _, e in ipairs({ -1, 1 }) do                                                                          -- rounded ends
					emit_box(side * (HW + (FL + BAR) / 2), SY + e * 0.4, z, (BAR - FL) / 2, 0.04, 0.07, lighten(shell, 0), 0)
				end
			end
		end
		face_bias = 0
		-- visor: smoked box tapering out from the front face
		local VY = my(-0.2075)                            -- visor centre height
		-- Visor: starts exactly at the head's front face and tapers out to the lenses. Above the face the
		-- shell curves back, so the visor's top and sides are extended back to meet that curve (on the
		-- model the dark part reaches y -0.039 at its top edge). Nothing of it sits inside the shell,
		-- so the painter's sort never has overlapping faces to fight over.
		-- (square-cornered where it meets the head, so it fills out to the top face and side pieces below
		-- without leaving gaps for the shell to show through; rounded at the front)
		emit_loft(VY, ZF, rrect(2.15, 1.0, 0.02, 2), mz(-0.128), rrect(1.92, 0.9, 0.28, 2), COL.trim, lighten(COL.trim, 8))
		do
			local VT = VY + 1.0                                  -- visor top edge
			-- the head's front curve between the front face and the visor's top edge (from PROF)
			local curve = { { mz(-0.070), my(-0.206) }, { mz(-0.066), my(-0.196) }, { mz(-0.060), my(-0.185) },
				{ mz(-0.050), my(-0.168) }, { mz(-0.040), my(-0.151) }, { mz(-0.039), VT } }
			local cz, cy = ZF, VT                                -- corner the visor fills out to
			local zb = curve[#curve][1]
			-- top face: from the curve back edge forward to the front face
			add_face({ { -2.15, VT, zb }, { 2.15, VT, zb }, { 2.15, VT, ZF }, { -2.15, VT, ZF } }, 0, VT - 0.5, (zb + ZF) / 2, COL.trim)
			-- side pieces: fan of triangles between the corner and the curve, on each side
			for side = -1, 1, 2 do
				local x = side * 2.15
				for i = 1, #curve - 1 do
					local a, b = curve[i], curve[i + 1]
					add_face({ { x, cy, cz }, { x, a[2], a[1] }, { x, b[2], b[1] } }, x - side, (cy + a[2] + b[2]) / 3, (cz + a[1] + b[1]) / 3, COL.trim)
				end
			end
		end

		local LZ = mz(0.024)
		emit_prism(0, H1 + 0.1, LZ, 0.3, 0.02, 12, lighten(shell, -40), 0, 0, lighten(shell, -40))   -- LED bezel
		-- lenses and slats on the visor front
		local VF = mz(-0.128)                                 -- visor front face
		face_bias = 1.8
		for side = -1, 1, 2 do
			emit_zprism(side * 1.25, VY + 0.06, VF + 0.03, 0.82, 0.06, 18, lighten(COL.trim, 22), 0, lighten(COL.trim, 22))  -- lens rim
		end
		for k = -2, 2 do emit_box(0, VY + 0.06 + k * 0.2, VF + 0.02, 0.42, 0.045, 0.02, lighten(COL.trim, 28), 0) end  -- slats
		face_bias = 2.6
		for side = -1, 1, 2 do
			emit_zprism(side * 1.25, VY + 0.06, VF + 0.08, 0.66, 0.06, 18, COL.lens, 0, lighten(COL.lens, 10))     -- lens glass
		end
		face_bias = 3.2
		for side = -1, 1, 2 do
			emit_box(side * 1.25 - 0.24, VY + 0.32, VF + 0.12, 0.09, 0.09, 0.01, { 190, 194, 210 }, 0)   -- glint
		end
		face_bias = 0
		end
		static_end()
		-- red LED on top of the head near the front edge of the plate: dark red when off, bright with a glow when on
		face_xf = head_xf
		do
			local blink = frame_no < blink_until and (frame_no // 4) % 2 == 0
			local led_on = R.led or blink
			local LZ = mz(0.024)
			if led_on then
				face_bias = 1.4
				emit_prism(0, H1 + 0.12, LZ, 0.34, 0.02, 12, { 255, 90, 70 }, 0, 0, { 255, 110, 90 })    -- glow
			end
			face_bias = 1.8
			emit_prism(0, H1 + 0.17, LZ, 0.17, 0.14, 10, led_on and { 255, 40, 30 } or { 120, 20, 22 }, 0, 0,
				led_on and { 255, 200, 180 } or { 150, 34, 34 })                                       -- LED
			face_bias = 0
		end

		face_xf = nil
		-- STACK-UP disks: stacked on the trays, or held between the grippers
		if game == "stackup" then
			for s = 0, 4 do
				local px, pz = station_pos(s)
				local st_ = stacks[s] or {}
				for i, b in ipairs(st_) do emit_block(px, SEAT + (i - 1) * DISC_H, pz, b.color, 0, i == #st_) end
			end
			for _, f in ipairs(falling) do                       -- groups dropping onto a stack
				local px, pz = station_pos(f.s)
				for i, b in ipairs(f.group) do emit_block(px, f.y + (i - 1) * DISC_H, pz, b.color, 0, i == #f.group, i == 1) end
			end
			for _, ev in ipairs(knocked) do                      -- knocked-off disks, tumbling / lying / carried
				for _, d in ipairs(ev.discs) do emit_disc_t(d.x, d.y, d.z, d.th, d.dir, d.color) end
			end
			if R.holding then                                    -- held group: bottom disk in the grippers
				local y0 = held_disks_y(yc)
				for i, b in ipairs(R.holding) do emit_block(0, y0 + (i - 1) * DISC_H, HOLD_R, b.color, yaw, i == #R.holding, i == 1) end
			end
		end

		-- GYROS
		for _, g in pairs(gyros) do
			if g.loc == "anim" then
				local A = g.anim
				emit_gyro_t(A.x, A.y, A.z, A.th, A.dir, g)
			elseif g.loc == "held" then
				-- The claws hold the gyro by the top of its spindle. Grabbed lower down, the claws slide up
				-- the spindle as the arms rise and only lift the gyro once they reach the top; lowered over
				-- a station, the gyro stands on it again and the claws slide back down the spindle.
				local hang = yc + 0.27 - (GYRO_TOP - GYRO_TIPL)                      -- disc height when hanging from the top
				local sb_ = station_of_body()
				local support = sb_ and seat_of(sb_) or 0.4                    -- station under the hands, or the floor
				g.last_gb = max(hang, support)                                 -- where it drops from if released
				emit_gyro(0, g.last_gb, HOLD_R, g, yaw, sb_)
			else
				local gx, gz = station_pos(g.station)
				local gb = seat_of(g.station, g.loc)   -- the gyro stands on its tip
				if g.falling then gb = g.fy else g.fy = nil end
				emit_gyro(gx, gb, gz, g, 0, g.station)
			end
		end

		-- the kept faces of the fixed parts (sorted once, when they are built) merged with this frame's
		-- own, sorted: the same painting order as sorting them all together, for a fraction of the work
		if build_static then
			table.sort(sfaces, face_order)
			SCACHE.key, SCACHE.faces = skey, sfaces
		end
		-- this frame's faces sort as whole numbers (depth to a millionth, then the order they were
		-- added in): the built-in comparison is far quicker than calling face_order for each pair
		local nd = #dfaces
		if nd < 65536 then
			local keys = {}
			for i = 1, nd do keys[i] = floor(-dfaces[i].depth * 1e6) * 65536 + i end
			table.sort(keys)
			local sorted = {}
			for k = 1, nd do sorted[k] = dfaces[keys[k] % 65536] end
			dfaces = sorted
		else
			table.sort(dfaces, face_order)
		end
		local all, kept = {}, SCACHE.faces
		local i, j, nk, nd, k = 1, 1, #kept, #dfaces, 0
		while i <= nk and j <= nd do
			local a, b = kept[i], dfaces[j]
			local da, db = a.depth, b.depth
			k = k + 1
			-- (face_order(b, a), inlined: a kept face's number is always lower than this frame's)
			if db > da then all[k] = b j = j + 1 else all[k] = a i = i + 1 end
		end
		table.move(kept, i, nk, k + 1, all)
		table.move(dfaces, j, nd, k + 1 + nk - i + 1, all)
		faces, faces_sorted = all, true
	end

	---------------------------------------------------------------------------
	-- panel drawing
	---------------------------------------------------------------------------
	-- everything the panel picture depends on, as one string
	function PCACHE.sig(sw, sh, px, py, pw, ph)
		local t = { game, machine_kind, tostring(cfg.mirror_lr), cfg.focal_scale, output_scale(), sw, sh, px, py, pw, ph,
			cam.yaw, cam.pitch, cam.ty, cam.dist, cam.tx or 0, cam.tz or 0,
			R.ang, R.h, R.claw, tostring(R.led), tostring(frame_no < blink_until and (frame_no // 4) % 2 == 0),
			tostring(R.holding), tostring(R.gyro_in), R.press.red or 0, R.press.blue or 0 }
		local keys = {}
		for k in pairs(gyros) do keys[#keys + 1] = tostring(k) end
		table.sort(keys)
		for _, k in ipairs(keys) do
			local g = gyros[k] or gyros[tonumber(k)]
			t[#t + 1] = k
			t[#t + 1] = tostring(g.loc) t[#t + 1] = tostring(g.station) t[#t + 1] = tostring(g.phase)
			t[#t + 1] = tostring(g.spin) t[#t + 1] = tostring(g.fy) t[#t + 1] = tostring(g.falling)
			local A = g.anim
			if A then t[#t + 1] = string.format("%s,%s,%s,%s,%s", A.x, A.y, A.z, A.th, A.dir) end
		end
		for s2 = 0, 4 do
			local st = stacks[s2]
			if st then
				local cs = {}
				for i, b in ipairs(st) do cs[i] = tostring(b.color or b) end
				t[#t + 1] = s2 .. ":" .. table.concat(cs, ",")
			end
		end
		for _, f in ipairs(falling) do t[#t + 1] = string.format("F%s,%s,%s", f.s, f.y, #f.group) end
		for _, ev in ipairs(knocked) do                    -- knocked-off disks: tumbling, lying, carried back
			for _, d in ipairs(ev.discs) do
				t[#t + 1] = string.format("K%s,%s,%s,%s,%s,%s", d.x, d.y, d.z, d.th, tostring(d.dir), tostring(d.color))
			end
		end
		if type(R.holding) == "table" and R.holding[1] then t[#t + 1] = "H" .. #R.holding end
		return table.concat(t, "|")
	end
	local function draw_panel()
		local sw, sh = screen_size()
		local pw, ph, m = cfg.panel_w, cfg.panel_h, cfg.panel_margin
		local px = cfg.panel_anchor:find("right") and (sw - pw - m) or m
		local py = cfg.panel_anchor:find("bottom") and (sh - ph - m) or m

		-- Most of the time R.O.B. is waiting and nothing in the picture moves: then the image painted last
		-- frame is handed to MAME again instead of being rebuilt. The key holds everything the picture
		-- depends on; a full redraw still happens at least once a second as a safety net.
		local key = PCACHE.sig(sw, sh, px, py, pw, ph)
		if tex_ok and ptex and PCACHE.quad and key == PCACHE.key and frame_no - PCACHE.at < 60 then
			local q = PCACHE.quad
			scr.container:draw_quad(ptex, q[1], q[2], q[3], q[4])
			return
		end
		build_scene()
		-- (R.O.B. moved right and down so he and his accessories sit in the panel's bottom-right corner)
		flush_faces(px + pw / 2 + 16 * cfg.focal_scale, py + ph / 2 + 24 * cfg.focal_scale, px, py, px + pw, py + ph)
		PCACHE.key, PCACHE.at = key, frame_no
	end

	---------------------------------------------------------------------------
	-- machine + cartridge detection
	---------------------------------------------------------------------------
	local function lower(v) return type(v) == "string" and v:lower() or "" end

	-- read a property that is a plain value in some MAME versions and a method in others
	local function prop(obj, name)
		local ok, v = pcall(function() return obj[name] end)
		if not ok then return nil end
		if type(v) == "function" then
			local ok2, r = pcall(v, obj)
			return ok2 and r or nil
		end
		return v
	end

	local GAME_KEYS = {
		{ "gyromite", { "gyromite", "robot gyro", "robotgyro", "robot_gyro", "robot-gyro" } },
		{ "stackup",  { "stack-up", "stack up", "stackup", "robot block", "robotblock", "robot_block", "block set" } },
		{ "xmas14",   { "xmas14" } },
	}

	-- returns kind ("nes"/"famicom"), game key or nil, and a text summary for the log
	local function detect()
		local kind, mname, mdesc = "nes", "", ""
		pcall(function()
			local sys = manager.machine.system
			mname, mdesc = lower(prop(sys, "name")), lower(prop(sys, "description"))
		end)
		-- NB: MAME's NES driver describes itself as "Nintendo Entertainment System / Famicom (NTSC)",
		-- so only the machine name, or a description that STARTS with Famicom wording, counts.
		if mname:find("^fam") or mname == "fds" or mdesc:find("^famicom") or mdesc:find("^family computer")
			or mdesc:find("^nintendo family computer") or mdesc:find("^nintendo famicom") then
			kind = "famicom"
		end

		local parts = {}
		pcall(function()
			for _, img in pairs(manager.machine.images) do
				if prop(img, "exists") ~= false then
					for _, f in ipairs({ "filename", "basename", "software_longname", "software_shortname" }) do
						local v = prop(img, f)
						if type(v) == "string" and v ~= "" then parts[#parts + 1] = v:lower() end
					end
				end
			end
		end)
		local blob = table.concat(parts, " | ")

		local found
		for _, entry in ipairs(GAME_KEYS) do
			for _, key in ipairs(entry[2]) do
				if blob:find(key, 1, true) then found = entry[1] break end
			end
			if found then break end
		end
		return kind, found
	end

	local function redetect()
		local ok, kind, found = pcall(detect)
		if not ok then return end
		local newgame = cfg.game ~= "auto" and cfg.game or found or cfg.fallback_game
		local newkind = kind
		if cfg.variant == "famicom" then newkind = "famicom" elseif cfg.variant == "us" then newkind = "nes" end
		if newgame ~= game or newkind ~= machine_kind or not configured then
			local game_changed = (newgame ~= game) or not configured
			game, machine_kind, configured = newgame, newkind, true
			apply_palette(machine_kind)
			if game_changed then setup_items(); R.holding = nil end
		end
	end

	---------------------------------------------------------------------------
	-- per-frame entry point
	---------------------------------------------------------------------------
	-- true from the moment a machine stops until the next one has been reset: nothing may touch
	-- MAME objects (screen, ioport, input) in between
	local machine_down = false
	local function machine_leaving()
		local ok, pending = pcall(function()
			local m = manager.machine
			return m.exit_pending or m.hard_reset_pending
		end)
		return ok and pending
	end

	local function on_frame()
		if machine_down or machine_leaving() then return end
		if not scr then
			scr = find_screen()
			if not scr then return end
			discover_ports()
		end
		frame_no = frame_no + 1
		-- MAME fills in the input ports a little after the first frame: keep looking until both
		-- tray buttons are mapped
		if (not btn_field.red or not btn_field.blue) and frame_no % 30 == 1 then discover_ports() end
		if frame_no == 1 or not configured or frame_no % cfg.detect_every == 0 then redetect() end

		local ok, bit, lit = pcall(classify_frame)
		if ok then decode_push(bit, lit) end

		sim_step()
		drive_button("red") drive_button("blue")
		pcall(draw_panel)
	end

	local first_start = true            -- the machine start right after the plugin loads is not a reset
	local function on_reset()
		machine_down = false
		if scr then redetect() end
		if first_start then first_start = false else reset_robot() end
		hist, last_decode_frame, alt_run, last_bit, blink_until = {}, -100, 0, nil, 0
		last_cmd = { name = "-", code = "----", frame = -1000 }
		for _, w in ipairs({ "red", "blue" }) do
			btn_state[w] = false
			if btn_field[w] then pcall(function() btn_field[w].field:clear_value() end) end
		end
	end

	init_robot()
	TEX.sticker = load_ppm("rob_sticker.ppm")
	TEX.sticker_fc = load_ppm("famicom_sticker.ppm")
	TEX.switch = load_ppm("switch_panel.ppm")
	TEX.gyro = load_ppm("gyro_label.ppm")
	TEX.block = load_ppm("block_label.ppm")
	TEX.spinner = load_ppm("spinner_label.ppm")

	-- frame callback: newer MAME uses notifier objects (keep a reference or they get collected)
	if emu.register_frame_done then
		emu.register_frame_done(on_frame, "frame")
	elseif emu.add_machine_frame_notifier then
		rob._frame_sub = emu.add_machine_frame_notifier(on_frame)
	end
	if emu.add_machine_reset_notifier then
		rob._reset_sub = emu.add_machine_reset_notifier(on_reset)
	elseif emu.register_start then
		emu.register_start(on_reset)
	end

	-- The machine is going away (hard reset, or new software chosen from the menu): forget every
	-- object that belongs to it. Using the old screen / input fields on the new machine crashes MAME.
	local function on_stop()
		machine_down = true
		release_panel_texture()
		for _, w in ipairs({ "red", "blue" }) do
			if btn_state[w] and btn_field[w] then pcall(function() btn_field[w].field:clear_value() end) end
			btn_state[w] = false
		end
		scr, pixel_mode, sample_points = nil, nil, nil
		p2_fields = {}
		btn_field.red, btn_field.blue = nil, nil
		configured = false
	end
	if emu.add_machine_stop_notifier then
		rob._stop_sub = emu.add_machine_stop_notifier(on_stop)
	elseif emu.register_stop then
		emu.register_stop(on_stop)
	end

	-- for testing without the game: the plugin module's command("1010") runs one R.O.B. command
	-- (here: open arms) given as its wxyz code
	exports.command = function(code)
		local cmd = COMMANDS[code]
		if not cmd then return end
		last_cmd = { name = cmd.name, code = code, frame = frame_no }
		on_command(cmd)
	end
end

return exports
