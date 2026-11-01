# Dependencies at both endpoints

All 16 vendor descriptors are byte-for-byte unchanged. GradleRIO stays 2026.2.1 and Java stays 17. No dependency declaration changed.

| Vendor | Version |
|---|---|
| AdvantageKit | 26.0.2 |
| Autopilot | 1.5.0 |
| BLine-Lib | v0.9.1 |
| DogLog | 2026.5.0 |
| LumynLabs | 2026.2.0 |
| PathplannerLib | 2026.1.2 |
| CTRE-Phoenix (v5) | 5.36.0 |
| CTRE-Phoenix (v6) | 26.1.2 |
| REVLib | 2026.0.5 |
| ReduxLib | 2026.1.2 |
| Studica | 2026.0.0 |
| ThriftyLib | 2026.1.2 |
| URCL | 2026.0.0 |
| WPILib-New-Commands | 1.0.0 |
| libgrapplefrc | 2026.0.0 |
| YAGSL | 2026.1.12 |

The YAGSL filename says 2026.1.11, but its declared library version is 2026.1.12 at both endpoints. The new outputs/teleop-controls/node_modules entry belongs to desktop spreadsheet tooling; it is not a Gradle runtime dependency.
