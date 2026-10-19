# Realistic simulator validation record

These are software/integration checks performed on the development x86_64 Linux
machine, 2026-10-15. The assembled robot is unfinished and has no calibration
recordings. None of these results establishes measured physical agreement.
Machine-readable results are in [validation.json](validation.json),
[source versions](versions.json), and the linked reports below.

## Default-path isolation and regressions

The original and final normal simulator, with realism compiled off and on,
produce the same 600-frame bincode/FNV-1a trajectory fingerprint:
`d051cf160ac23c84`. Layouts remain `World=3440`, `PhysicsWorld=2608`,
`RobotSim=160` bytes. Default features are empty and the standalone normal
runtime dependency tree contains no `toml`, `sha2`, or `robot-command-receiver`.
Ordinary stepping and Rapier dependency features are unchanged. Local probe
runs took 0.037/0.040 seconds; that difference is timing noise, not an added
modeled workload or a performance guarantee.

Reproduce against the actual library source:

```sh
SIMHARK_SKIP_FRONTEND=1 cargo run -r --manifest-path tools/baseline-runner/Cargo.toml \
  --config .cargo/local.toml
SIMHARK_SKIP_FRONTEND=1 cargo run -r --manifest-path tools/baseline-runner/Cargo.toml \
  --config .cargo/local.toml --features realistic
cargo tree --manifest-path tools/baseline-runner/Cargo.toml --config .cargo/local.toml -e normal
```

Normal 60-second match regressions completed: Bangka versus Ungabunga at seed 3
(0–0, Bangka on evaluation points), versus Ungabunga at seed 7 (0–0, Ungabunga
on points), and versus Bongka at seed 3 (0–0, Bongka on points). These are smoke
regressions, not evidence of good soccer or hardware fidelity. The underlying
Bongka/Ungabunga implementations were not changed.

The optional Division B full-profile match ran 60 seconds with actual Bangka,
Ungabunga and referris: **Yellow Bangka 1–0 Blue Ungabunga**. This uses team
observations and a separate officials tracker. A 10-second paced match against
actual Sumatra completed **0–0**, Sumatra ahead on evaluation points, with no
AI exceptions after copying its geometry/robot settings into the isolated
launch directory. Java 25 is required by this Sumatra build; Java 21 correctly
fails with an unsupported-class-version error.

## Test and replay checks

The standalone actual-source suite passes 19 timing/receiver/physics tests and
32 full-system tests. Core checks pass 24 unit tests by default and 27 with realism enabled, plus
41 integration tests in both modes (one existing external integration ignored).
Two Sumatra protocol tests and five standalone shared receiver tests pass.
Clippy completes; style warnings remain in existing and new code, and upstream
sibling crates also emit warnings. No correctness lint failure was reported.

Coverage includes native rates/causality, clocks/FIFO/wrap/reset, processing
completion/overload, correlated stalls/expiry/reorder/deduplication/safety,
independent wheel slip/current/latch/reset, free-3D support loads and gravity
tipping, step halving during drive/reversal, physical venue collision, power
sag/restart/thermal latch, delayed-kick cancellation, magnetic unpowered rotor
and mount effects, camera projection/occlusion/audience/processing, stereo ball
height ambiguity, SS/DS native ranging, TDoA yaw ambiguity, no-overhead
localization/kidnapping/restart, local ball sensing versus video, standard UDP
commands/setup/malformed input, calibration fits/held-out failures, and profile
layer/digest persistence.

Built-in recording/replay verifies events, diagnostics and final hardware/truth
exactly. A real spawned Rust JSONL surrogate controller ran for one paced second;
its 2,002 control ticks included 664 nonzero-output ticks, then replayed exactly
from recorded controller outputs without rerunning the process. Wall-clock
process arrival order is recorded rather than called deterministic.

## Production fusion and native firmware

[Actual vIMU report](actual-vimu.json) exercises the existing Crashanter adaptive
X16/BMI fusion directly. All eight runs complete: healthy, missing, frozen,
clipped, clock reset, duplicate, reordered and wrong-delay input. Healthy yaw
RMS is approximately 0.00073 rad/s in this synthetic maneuver; deliberately
wrong delay raises it to approximately 0.0131 rad/s. These are synthetic
self-consistency results. They are not measured robot estimation accuracy.
Crashanter's working tree already contains user changes; the exact used source
hashes accompany its HEAD in `versions.json`.

[Native firmware report](native-firmware.json) records **159,912 calls** to the
unchanged TIGERs motor2019 fixed-point current PI, plus the native speed PI,
through quantized Rust encoder/ADC interfaces at 1/20 kHz. The adapter pins the
actual C source/header hashes, compiles unchanged code and checks its ABI. The
physical step is 25 µs; ADC availability is delayed. Gains and plant are
provisional. This is a scalar averaged q-axis HAL, not full STM32 instruction,
Park/PWM, device-register or board firmware emulation.

## Actual SSL-Vision detector

The unchanged official SSL-Vision checkout at
`5321d7bd995e7cc584d1b1937818e3f5ea2c9fae` was built and run under Xvfb/llvmpipe.
The Rust renderer exported actual 1920×1200 BMP images, focal length 900 px,
with calibrated fixture poses and physical SSL2010_16 robot markers. SSL-Vision
read those files and produced real multicast protobuf detections. No truth
objects were inserted into its output.

[Default-filter capture](ssl-vision-default-filters.json): 273 detection frames,
both robot IDs/headings recognized, **zero balls accepted**. Example robot
positions were approximately (−2251.3, 2.7) mm and (2280.6, 2.7) mm for scene
centers (−2250, 0) and (2250, 0) mm. Team-height/detection/calibration assumptions
therefore remain visible in the residuals.

[Histogram-disabled diagnostic](ssl-vision-histogram-disabled.json): disabling
only `Thread 0 / Ball Detection / Histogram Filter / Enable Filter` in the
isolated detector settings yielded 254 ball detections in 254 frames, near
(−5.1, 2.8) mm for the ball at (0, 0). Other detector filters and source code
were unchanged. The generated fixture preserves default filters. This diagnostic
isolates an image/LUT/histogram calibration failure; it does not assert the
same setting is appropriate for a real camera. Both outcomes are retained.

The actual detector also fed 217 UDP frames (all with ball detections) into a
paced four-second `RealisticWorld` run with synthetic vision disabled. The
recorded external inputs replayed exactly. The input images repeated an exported
sequence, so this validates the live ingress/recording boundary, not agreement
between those images and the concurrently moving simulated scene.

The local build used Nix Eigen 3.4.1/OpenCV 4.13/Qt5/protobuf dependencies with
`USE_DC1394=OFF`, `USE_V4L=OFF`, `CMAKE_BUILD_TYPE=Release`,
`-DNO_DC1394_CONVERSIONS` and linker `--copy-dt-needed-entries`. Host Mesa/Xvfb
provided software GL. Release mode was needed for an upstream OpenCV vector
indexing assertion; the SSL-Vision source was not patched. Image file reading
runs unpaced and repeats its sequence; detector timestamps and CPU duration are
wall-clock results, not a deterministic simulated camera-rate claim.

## Real logs and calibration workflow

[Complete log analysis](camera-log-analysis.json) used:

- `2026-07-05_12-32_ELIMINATION_PHASE_RobôCin-vs-TIGERs_Mannheim.log.gz`,
  1,260,633 records, canonical SHA256
  `045863a5eaf1d66eada7135d8d35568b33fc940d37d7fbd13e874310e59ea198`.
- `2025-07-20_14-31_ELIMINATION_PHASE_The_Bots-vs-ITAndroids.log.gz`, canonical
  SHA256 `1f57cb7215bf9d98a91341cf57c06586e6b650e5a57e0ce478d0a4b160468890`.

These were complete files, not a bounded prefix. Camera-0 native interval KS
was approximately **0.9993**, showing strong venue/rate distribution shift;
matching camera IDs do not establish matching hardware. The CLI also exported
a real native-interval/processing-duration timing prior and successfully ran it
through the simulator. Geometry, optical noise and robot hardware were not
fitted from those logs. Processing priors include their observed queueing span.

Motor RL/back-EMF, joint copy timing, and sensor delay/bandwidth/scale/bias fitting
are tested with explicitly synthetic, independently identified fixtures.
Unexcited data, unknown clock boundaries, inconsistent UWB air-end timing and
failed held-out tolerances are rejected. Exported synthetic overlays remain
provisional. Future hardware imports use the same code and the empty templates
in `profiles/realistic/calibration`; see the [calibration guide](../../realistic-calibration.md).

## Checkouts and reproducibility

Primary sources used locally:

| Component | Checkout / source |
| --- | --- |
| Crashanter | `../Crashanter`, HEAD plus source hashes in versions.json |
| Firmware | `../Firmware`, https://github.com/TIGERs-Mannheim/Firmware |
| SSL-Vision | `/tmp/simhark-ssl-vision`, https://github.com/RoboCup-SSL/ssl-vision |
| Sumatra | `../Sumatra`, revision 496376d77d9e208736257f4410ad1dd5ca9cf72d |
| referris | `../referris`, revision 579de71487e8fcb84aaa657c233b5e81a45c5325 |
| Legacy CrashPilot for workspace validation | `../CrashPilot-simhark-realism-validation`, detached ecb8fd8c8b6252a3ae4f09bacca72f3bcdf3394e |

The current `../CrashPilot` has removed APIs/features required by existing FAABS
workspace members. It was left untouched. A separate detached worktree at the
compatible commit enabled full workspace checks. `/tmp/simhark-legacy-local.toml`
contains the existing `.cargo/local.toml` patches resolved to absolute sibling
paths, with only CrashPilot redirected to that validation worktree. Root test
and match commands used this replacement config; independent tools used the
ordinary `.cargo/local.toml`. Validation targets were outside the repository.

Run the tests listed in the main usage document. Match validation used
`--features realistic,sumatra,referris,viewer,viewer-debug`; set
`SIMHARK_SUMATRA_REPO_ROOT` to the Sumatra checkout and `JAVA_HOME` to Java 25.
Native firmware uses `SIMHARK_FIRMWARE_ROOT` or the sibling `Firmware` by default.
Exact external runtime versions are recorded, not assumed to be the current
versions of moving upstream branches.
