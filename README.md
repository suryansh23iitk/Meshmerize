---

# 🧠 Meshmerize Bot – IIT Bombay Techfest (Zonal: Jaipur)

An autonomous line‑follower robot built for **Meshmerize**, a major event in the Techfest zonals, where teams must map a maze during a dry run and complete the optimal path in a final run. The code implements PID-based line following, real-time turn detection, path recording, simplification, and replay on ESP32-drivetrain hardware.

---

## 🎯 Event Context: Meshmerize (Line‑Follower Bot)

**Meshmerize** is an autonomous line‑follower event under Techfest, IIT Bombay’s annual technology fest. In zonal rounds such as those in Jaipur, participants program robots to follow white lines through complex mazes:

* **Dry run**: map the maze—bot records turns (“L”, “R”, “B” for U-turn, “F” for forward).
* **Final run**: replay the shortest path to the finish. ([events.vtools.ieee.org][1], [en.wikipedia.org][2])

Participation is high—typically 150+ students in zonals—and top teams earn certificates and advance to national finals. ([events.vtools.ieee.org][1])

---

## 🏆 Achievements

* Gold medal in **Jaipur zonals** of the Meshmerize event — congratulations!
* Secured victory among stiff competition, earning top honors and qualification for the next stage.
* Best Run time of **(0:58) dry run** and **(0:28) final run**.

---

## ⚙️ Code Overview

### Hardware Setup

* **Motors**: Dual-motor control using a motor driver (pins AIN1/AIN2/PWMA & BIN1/BIN2/PWMB; STBY for standby).
* **Sensors**:

  * 6 middle-line sensors (analog): `sensorArray[6] = {34, 25, 14, 35, 26, 27}`;
  * 2 corner sensors (interrupt): INT0 (pin 39), INT1 (pin 36) detect turns.
* **Controls**:

  * LED (pin 2) for status;
  * Push buttons SW1 (pin 22) to toggle "left-turn-first" mode; SW2 (pin 23) to initiate final run.

### Flow Logic

1. **dry run** (in `loop()`):

   * Line-following using `PID()`;
   * Turn detection via interrupts sets `flag_turn`;
   * Turn handling (`Turn_Left_Priority()` or `Turn_Right_Priority()`) based on mode; record turns to `path[]`;
   * Detect U-turns (all sensors black) and record 'B'.
   * Use `simplify_path()` to remove redundant loops.

2. **final run**:

   * After reaching the end and pressing SW2:
   * Bot replays `path[]`, taking each recorded turn in sequence;
   * Includes braking and forward-thrust maneuvers to align correctly;
   * Stops at end-of-path (‘E’).

### Core Features

* **PID line-following**: Efficient real-time control adjusting left/right motor speeds proportionally and based on derivative (`Kd`) of error.
* **Interrupts for turns**: Corner sensors detect crossings to trigger turn logic.
* **Path recording**: Stores sequence of 'L', 'R', 'B', 'F', 'E' in `path[]`.
* **Path simplification**: Removes back-and-forth loops using angle-sum method.
* **Mode selection**: Choose left-first or right-first priority via button.
* **Final run autonomously replays** shortest path.

---

## 📋 File Structure

Everything is contained in `main.ino` (ESP32 Arduino code).

**Key functions**:

* `setup()` – initialize pins, interrupts, serial port.
* `loop()` – toggle between dry and final runs.
* `PID()` / `linefollow()` – maintain line alignment.
* `Turn_Left_Priority()` / `Turn_Right_Priority()` – resolve turns during dry run.
* `Left()`, `Right()`, `Backward()` – perform different turn types.
* `simplify_path()` – optimize recorded path after each turn.
* `GetSensorData()` – refresh all sensor readings.
* `TurnAvailable()` – interrupt handler marking new turn.

---

## 🧩 Usage Instructions

1. Upload to ESP32 connected to sensors and motor-driver.
2. Dry run by powering on—bot follows maze, maps path.
3. Press **SW1** to toggle left/right priority if needed.
4. Once maze end is detected, LED turns on; record path displayed via Serial.
5. Press **SW2** to start the final run—bot autonomously executes the optimal path.
6. Final path playback completes; LED indicates when finished.

---

## 📈 Performance Notes

* **PID tuning**: `Kp = 0.04`, `Kd = 3.0`, `Ki = 0.1`, basespeed 140, max\_speed 190. Provides stable in-maze tracking.
* **Interrupt-driven turn detection** enables precise corner localization.
* **Path simplification** reduces unnecessary detours, improving final-run efficiency.
* **Final run strategy** uses braking & forward thrusts to align before turns, reducing overshoot.

---

## 💡 Future Improvements

* Add **self-tuning PID** or dynamic speed adjustment.
* Use **RGB LEDs** for visual debugging of turn types.
* Add **auto final-run trigger**, skipping SW2 manual press.
* Support larger maze or **dynamic obstacle detection**.

---

## 📝 Credits

* Code written by \[Rahul Singh/Suryansh Rastogi/Lipika Bagai].
* Event held as part of Techfest, IIT Bombay—Zonals in Jaipur (Oct 2024) 
* Meshmerize: line-follower maze competition requiring mapping then fastest final run
* Gold Medal achieved—congratulations once more!

---

