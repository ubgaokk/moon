# Moon - FTC Robot Drive System

Full FTC Robot Controller project with drive system extracted from DECODE V2.

## Structure

```
moon/
├── FtcRobotController/           # Official FTC SDK module
├── TeamCode/
│   └── src/main/java/org/firstinspires/ftc/moon/
│       └── drive/
│           ├── MoonRobot.java    # Core drive + PedroPathing localization
│           └── MoonTeleOp.java   # TeleOp program
├── build.common.gradle           # Standard FTC SDK build config
├── build.dependencies.gradle      # Dependencies
├── build.gradle                  # Top-level project config
├── settings.gradle               # Module includes
├── gradle/
└── gradlew
```

## Build

Open in Android Studio and build like any FTC project:

```bash
./gradlew build
```

## Key Settings (Same as DECODE V2)

- **Motor names**: W_FL, W_BL, W_BR, W_FR
- **Motor directions**: FL/BL reversed
- **Path constraints**: velocity=0.99, accel=100, jerk=1, heading_vel=1.5
- **PID constants**: transl P=0.08, heading P=1, drive P=0.015
- **Pinpoint localizer**: enabled (GoBilda 4-bar pod)
- **Mass**: 12.066 kg
- **Max velocity**: x=68, y=55.8 in/s

## Dependencies

- `com.pedropathing:ftc:2.0.4`
- `com.pedropathing:telemetry:1.0.0`
- `com.acmerobotics.dashboard:dashboard:0.2.4+0.4.17`
- `com.acmerobotics.roadrunner:...`

## TeleOp Controls

| Input | Action |
|-------|--------|
| Left Stick | Field-relative forward/strafe |
| Right Stick X | Rotation |
| LB (g1) | Intake toggle |
| LB (g2) | Slow mode |
| RT (g2) | Stopper open |
| LT (g2) | Stopper close |
| RB (g2) | Shoot request |
| X | Toggle setPose |
| Y | Toggle move shot |
| Start | Relocalize |

## License

MIT