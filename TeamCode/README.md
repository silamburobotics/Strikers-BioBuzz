# BioBuzz FTC Robotics - Robot Controller & TeleOp Documentation

This repository contains the official FTC TeleOp OpModes, subsystem libraries, and vision tracking code for the **BioBuzz FTC Robot**.

---

## 🏆 Driver Station OpModes Summary

On the REV Driver Station drop-down menu, select the OpMode corresponding to your team and alliance:

| Team | Alliance | OpMode Name | File Path |
| :--- | :--- | :--- | :--- |
| **Warriors** | **Blue Alliance** | `BioBuzz Centric BLUE Warriors` | [`BioBuzzCentricBlueWarriors.java`](file:///Users/karthiksadhasivam/Projects/FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzCentricBlueWarriors.java) |
| **Warriors** | **Red Alliance** | `BioBuzz Centric RED Warriors` | [`BioBuzzCentricRedWarriors.java`](file:///Users/karthiksadhasivam/Projects/FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzCentricRedWarriors.java) |
| **Strikers** | **Blue Alliance** | `BioBuzz Centric BLUE Strikers` | [`BioBuzzCentricBlueStrikers.java`](file:///Users/karthiksadhasivam/Projects/FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzCentricBlueStrikers.java) |
| **Strikers** | **Red Alliance** | `BioBuzz Centric RED Strikers` | [`BioBuzzCentricRedStrikers.java`](file:///Users/karthiksadhasivam/Projects/FtcRobotController/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzCentricRedStrikers.java) |

---

## ⚔️ Warriors vs. Strikers Gamepad Comparison Tables

### 🎮 Gamepad 1: Driver Controls Comparison

| Control Input | Warriors (`BioBuzzCentricBlue/RedWarriors`) | Strikers (`BioBuzzCentricBlue/RedStrikers`) |
| :--- | :--- | :--- |
| **Left Stick Y** (Drive) | `y = -gamepad1.left_stick_y`<br>• Push UP $\rightarrow$ Forward<br>• Pull DOWN $\rightarrow$ Backward | `y = -gamepad1.left_stick_y`<br>• Push UP $\rightarrow$ Forward<br>• Pull DOWN $\rightarrow$ Backward |
| **Left Stick X** (Strafe) | `x = gamepad1.left_stick_x`<br>• Push RIGHT $\rightarrow$ Strafe Right<br>• Push LEFT $\rightarrow$ Strafe Left | `x = gamepad1.left_stick_x`<br>• Push RIGHT $\rightarrow$ Strafe Right<br>• Push LEFT $\rightarrow$ Strafe Left |
| **Right Stick X** (Turn) | **`rx = -gamepad1.right_stick_x`**<br>• Standard Inverted Turn Direction | **`rx = gamepad1.right_stick_x`**<br>• **Reversed Turn Direction** |
| **Button A** | Toggle Intake Motor **FORWARD / STOP** | Toggle Intake Motor **FORWARD / STOP** |
| **Button B** | Toggle Intake Motor **REVERSE / STOP** | Toggle Intake Motor **REVERSE / STOP** |
| **Left Bumper (`LB`)** | Toggle Dual Intake Lift Servos **UP / DOWN** | Toggle Dual Intake Lift Servos **UP / DOWN** |

---

### 🕹️ Gamepad 2: Operator Controls Comparison

| Control Input | Warriors (`BioBuzzCentricBlue/RedWarriors`) | Strikers (`BioBuzzCentricBlue/RedStrikers`) |
| :--- | :--- | :--- |
| **Button B** | **Axon Gate Servo OPEN**<br>• Target Servo Position: `0.4` | **Axon Gate Servo OPEN**<br>• Target Servo Position: `0.4` |
| **Button X** | **Axon Gate Servo CLOSED**<br>• Target Servo Position: `0.7` | **Axon Gate Servo CLOSED**<br>• Target Servo Position: `0.7` |
| **Button A** | **Toggle Hood Shifter Servo**<br>• **OPEN** *(Nectar Mode)*: Servo `0.25`, Preset `2520 RPM`<br>• **CLOSED** *(Pollen Mode)*: Servo `0.95`, Preset `2420 RPM` | **Toggle Hood Shifter Servo**<br>• **OPEN** *(Nectar Mode)*: Servo `0.25`, Preset `2520 RPM`<br>• **CLOSED** *(Pollen Mode)*: Servo `0.95`, Preset `2420 RPM` |
| **Left Bumper (`LB`)** | **Toggle Turret Auto-Aim ON / OFF**<br>• Starts **ON** by default (`autoAimActive = true`)<br>• **Blue Alliance**: Filters Tags **39, 40, 43, 44** (and 1, 2, 3)<br>• **Red Alliance**: Filters Tags **31, 32, 35, 36** (and 4, 5, 6) | **Toggle Turret Auto-Aim ON / OFF**<br>• Starts **ON** by default (`autoAimActive = true`)<br>• **Blue Alliance**: Filters Tags **39, 40, 43, 44** (and 1, 2, 3)<br>• **Red Alliance**: Filters Tags **31, 32, 35, 36** (and 4, 5, 6) |
| **Button Y** | **Toggle Shooter Flywheels ON / OFF**<br>• Starts **OFF** by default for safety<br>• **Auto-Aim Active**: Calculates RPM dynamically based on distance<br>• **Master Motor 1**: Closed-Loop Velocity PID (`setVelocity`)<br>• **Slave Motor 2**: Power-Synced Follower (`shooter2.setPower(shooter1.getPower())`) | **Toggle Shooter Flywheels ON / OFF**<br>• Starts **OFF** by default for safety<br>• **Auto-Aim Active**: Calculates RPM dynamically based on distance<br>• **Master Motor 1**: Closed-Loop Velocity PID (`setVelocity`)<br>• **Slave Motor 2**: Power-Synced Follower (`shooter2.setPower(shooter1.getPower())`) |

---

## 🤖 Subsystems & Mechanism Architecture

### 1. Intake System
* **Motor**: `intake` (DcMotorEx, 100% duty cycle power `INTAKE_POWER = 1.0`).
* **Synchronized Lift Servos**: `intakeLiftLeft` and `intakeLiftRight`.
  * **UP Position**: `0.40`
  * **DOWN Position**: `0.80`
  * Right servo is automatically mirrored in real-time with an optional trim offset (`RIGHT_SERVO_TRIM_OFFSET`).

### 2. Axon Gate & Hood Shifter Servos
* **Axon Gate Servo**: `gateServo`
  * **Open**: `0.4` | **Closed**: `0.7`
* **Axon Shifter Servo**: `shifterServo`
  * **Open (Nectar Mode)**: `0.25`
  * **Closed (Pollen Mode)**: `0.95`

### 3. Turret Auto-Aim Mechanism
* **Motor**: `turret` (GoBILDA 312 RPM Yellow Jacket motor).
* **Gear Ratio**: $175:77 = 2.2727:1$ ($2.2727$ motor revolutions per $1.0$ turret revolution).
* **Velocity Capping**: Controlled via `RUN_TO_POSITION` with max speed capped at `90.0°/sec` and max power capped at `0.35` (35%) for smooth movement.
* **Soft Limits**: Enforced strictly between $-90.0^\circ$ and $+90.0^\circ$ from center home ($0.0^\circ$).

### 4. Master-Slave Flywheel Shooter Motors
* **Master Motor 1 (`flyWheel1`)**:
  * Connected to a 4-pin JST encoder cable.
  * Runs closed-loop velocity PID (`DcMotor.RunMode.RUN_USING_ENCODER`).
  * Dynamically calculates target velocity in ticks/sec to lock onto exact target RPM.
* **Slave Motor 2 (`flyWheel2`)**:
  * Operates without an encoder cable (`DcMotor.RunMode.RUN_WITHOUT_ENCODER`).
  * Synchronized directly to Master Motor 1's power output (`shooter2.setPower(shooter1.getPower())`).
  * Eliminates motor overheating and eliminates runaway voltage.

#### Preset Target Speeds
* **Pollen Mode (Shifter Closed)**: `POLLEN_BASE_RPM = 2420.0 RPM`
* **Nectar Mode (Shifter Open)**: `NECTAR_BASE_RPM = 2520.0 RPM`

---

## 📷 Vision Subsystem (`LimelightShooterSubsystem`)

The vision subsystem automatically detects whichever camera is connected to the robot:

### Dual Camera Auto-Detection
1. **Limelight 3A** (Priority 1): Native pipeline streaming at 15 FPS.
2. **Logitech C270 USB Webcam** (Priority 2): Uses FTC `VisionPortal` & `AprilTagProcessor` (`"Webcam 1"`).

### Alliance AprilTag Filtering
* **Blue Alliance Tags**: `39`, `40`, `43`, `44` (and Audience/Backdrop Tags `1`, `2`, `3`, `11`, `12`, `13`).
* **Red Alliance Tags**: `31`, `32`, `35`, `36` (and Audience/Backdrop Tags `4`, `5`, `6`, `14`, `15`, `16`).
* Opposing alliance tags and unapproved targets are strictly filtered out.

### Dynamic Distance RPM Calculation
$$\text{Calculated Target RPM} = \text{BASE\_RPM} + (\text{SLOPE} \times \text{AprilTag Distance in Inches})$$

---

## 💻 FTC Dashboard Tuning (`http://192.168.43.1:8080/dash`)

Connect your laptop to the Robot Controller Wi-Fi and open `http://192.168.43.1:8080/dash`:

1. **Live Camera Stream**: Select the camera view widget in the top-right corner to view live 15 FPS AprilTag video overlays.
2. **Horizontal Target Offset (`TURRET_AIM_OFFSET_DEG`)**: Expand `LimelightShooterSubsystem` to adjust horizontal alignment so shots land dead-center in the basket.
3. **Base RPMs**: Adjust `POLLEN_BASE_RPM` or `NECTAR_BASE_RPM` live without re-compiling code.

---

## 🔌 REV Hardware Map Configuration Names

Ensure these exact device names are configured in the REV Driver Station Robot Configuration:

| Hardware Device | Type | Configured Name |
| :--- | :--- | :--- |
| **Front Left Drive Motor** | `DcMotorEx` | `leftFront` |
| **Back Left Drive Motor** | `DcMotorEx` | `leftBack` |
| **Front Right Drive Motor** | `DcMotorEx` | `rightFront` |
| **Back Right Drive Motor** | `DcMotorEx` | `rightBack` |
| **Intake Motor** | `DcMotorEx` | `intake` |
| **Intake Left Lift Servo** | `Servo` | `intakeLiftLeft` |
| **Intake Right Lift Servo** | `Servo` | `intakeLiftRight` |
| **Axon Gate Servo** | `Servo` | `gateServo` |
| **Axon Hood Shifter Servo** | `Servo` | `shifterServo` |
| **Turret Motor** | `DcMotorEx` | `turret` |
| **Master Shooter Motor 1** | `DcMotorEx` | `flyWheel1` |
| **Slave Shooter Motor 2** | `DcMotorEx` | `flyWheel2` |
| **Limelight Camera** | `Limelight3A` | `limelight` |
| **Logitech USB Webcam** | `WebcamName` | `Webcam 1` |
