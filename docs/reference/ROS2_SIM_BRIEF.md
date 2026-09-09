# QDrone2 flight stack — integration brief for the ROS2 simulator

Context for an agent building a ROS2 drone simulator that must interoperate with
the QDrone2 hardware flight stack. Written 2026-08-31.

Every claim below is labelled:
**[MEASURED]** confirmed on hardware · **[REFERENCE]** from vendor code, unverified ·
**[SUSPECT]** believed wrong, a test exists to settle it

---

## 1. The integration contract (read this first)

**The flight stack does not use ROS.** It receives localization over a UDP
socket. That is the entire seam.

If your simulator emits the packet below to `127.0.0.1:47900`, the flight stack
cannot distinguish sim from real hardware. No ROS version negotiation, no
message-type coupling, no rebuild. **Match this packet and you are done.**

### Packet: 384 bytes, little-endian, naturally packed (no padding)

| offset | type | field | notes |
|---|---|---|---|
| 0 | `uint32` | `magic` | **`0xF12D6011`** — receiver drops anything else |
| 4 | `uint32` | `seq` | monotonic; gaps are logged as drops |
| 8 | `int64` | `t_ns_epoch` | **CLOCK_REALTIME** nanoseconds since Unix epoch |
| 16 | `double[3]` | `pos` | world frame, metres |
| 40 | `double[4]` | `quat` | **`qw, qx, qy, qz`** — Hamilton, world←body |
| 72 | `double[36]` | `cov_pose` | 6×6 row-major, order `[x,y,z,roll,pitch,yaw]` |
| 360 | `double[3]` | `vel` | **world/global frame** linear velocity, m/s |

**The receiver rejects any datagram whose size is not exactly 384 bytes.** Check
your struct packing.

Secondary port `47901` carries `/cloud_registered` point cloud, chunked, after a
5 m distance filter and 5 cm voxel downsample. Optional — not needed for hover.

### Timestamp handling — matters more than it looks

`t_ns_epoch` is wall-clock, but the flight stack runs on `steady_clock`. On every
packet it recomputes the offset:

```
delay      = system_clock::now() - t_ns_epoch
t_internal = steady_clock::now() - delay
```

So sender and receiver must share a realtime clock (they're on localhost today).
**Consequence for you:** stamp the packet at the moment the pose is *valid*, not
when you send it. Any latency you add here is indistinguishable from sensor
latency, and see §5 — latency is a live suspect in an unsolved bug.

Staleness threshold is **0.5 s**. Past that the estimator declares SLAM lost and
falls back to hold-position with zero velocity.

---

## 2. What currently produces that packet

**Current stack is ROS1, not ROS2.** [MEASURED — it is what runs today]

```
Livox Mid-360S
  └─ livox_ros_driver2  (msg_MID360s.launch)     → /livox/lidar, /livox/imu
      └─ fast_lio       (mapping_mid360.launch)  → /Odometry
          └─ fastlio_udp_bridge (custom, ~50 lines) → UDP :47900
```

catkin workspace at `/home/nvidia/fastlio_ws/`. `livox_ros_driver2` supports
ROS2; FAST-LIO2 has ROS2 forks. If you port, **keep the UDP bridge as-is** — it's
the stable interface, and it's the reason a port doesn't touch flight code.

`vel` is FAST-LIO's EKF `state_point.vel`, not a position difference. The flight
stack uses it directly and prefers it over differentiating position. If your sim
publishes velocity, publish a real one — a numerically differentiated pose will
change the closed-loop behaviour.

License note: FAST-LIO2 is GPL-2.0. Academic use assumed.

---

## 3. Airframe and hardware

| item | value | confidence |
|---|---|---|
| Airframe | Quanser QDrone2, 4 rotors | |
| Compute | NVIDIA Jetson, onboard | |
| IO | Quanser HIL card (QUARC SDK) | |
| LiDAR | Livox Mid-360S | |
| Rangefinder | VL53L1X, downward, I²C | |
| IMU | Quanser onboard, gyro + accel | |

### Physical parameters

| parameter | value | confidence |
|---|---|---|
| mass | **1.8 kg** | **[SUSPECT]** see below |
| `L_roll` (roll moment arm) | 0.2136 m | [REFERENCE] |
| `L_pitch` (pitch moment arm) | 0.1758 m | [REFERENCE] |
| `k_tau` (yaw torque ↔ force) | 81.0363 N/N·m | [REFERENCE] |
| `I_xx`, `I_yy`, `I_zz` | 0.0124, 0.0106, 0.0150 kg·m² | [REFERENCE] not measured |
| hover thrust | 17.66 N @ 1.8 kg | derived |
| `f_max` | 25 N | [REFERENCE] |

> **Mass is contested.** Three values exist in the old tree from what looks like
> one weighing session: `1.495` (a struct default), `1.7` (a comment), `1.8`
> (config, authoritative at runtime). 1.495 vs 1.8 is a 17% hover-thrust error.
> **Use 1.8 for now, but treat it as provisional** — it is being re-weighed.

### Motor layout — **[SUSPECT]**

```
   front
M1 ---- M3      M0 = back-right,  CW
 |  ^   |       M1 = front-right, CCW
 |  |x  |       M2 = back-left,   CCW
M0 ---- M2      M3 = front-left,  CW
   back
```

**Inferred from ESC direction commands, never confirmed visually.** Every mixer
sign depends on it. A bench test (`t1b_motor_id`) is scheduled to settle it. If
your sim hardcodes this layout, flag it as an assumption.

---

## 4. Sensors and actuators, for modelling

### IMU [REFERENCE]
- gyro: body frame, **rad/s**, ±250 °/s FS, 500 Hz, 125 Hz BW
- accel: body frame, **m/s², gravity included**, ±16 g, 1000 Hz, 400 Hz BW
- no magnetometer → **yaw is unobservable and drifts**. The flight stack
  integrates gyro-z and accepts the drift. Model this; don't hand it a perfect yaw.

### Rangefinder [REFERENCE]
- VL53L1X, MEDIUM profile (~2.9 m), 50 ms timing budget, 60 ms period → **~16 Hz**
- most reads return no new data; that's normal, not an error

### Motors / ESC [REFERENCE for protocol, SUSPECT for thrust]
- DShot1200 over PWM channels 0-3 in RAW mode
- throttle values: `0` = stop/disarm, `48` = armed idle (no rotation),
  `~100` = spin threshold, `2047` = max
- **~3.15 s arm sequence** before any throttle is accepted. If your sim models
  the ESC, model this delay — the flight stack blocks on it at startup.

### Thrust model — **[SUSPECT], do not trust**

The vendor analytic model:
```
Fm = C_t*(w + w_f)^2 + F_b ,  w = K_v_eff*V + w_c ,  V = V_d * pwm
C_t=2.0784e-8  F_b=-0.2046  w_f=1004.5  w_c=2132.6  K_v_eff=1295.4  V_d=12.6
```

**This model has never been checked against a scale, and measured evidence says
it overestimates delivered thrust** (see §5). A bench sweep is scheduled to
replace it with a fit. **If your sim uses these constants, your sim will lift
when the real airframe does not.** Consider parameterising the thrust curve so
the measured fit can be dropped in.

---

## 5. Two unsolved failures — please model these, don't paper over them

The previous flight stack ran for a month and failed in two ways. Both are
**below the control law**, so a simulator with an idealised actuator or an
idealised latency will hide them.

### (a) Will not lift [MEASURED]
Commanded thrust 19.3 N against a 17.66 N hover requirement, no saturation, no
battery cutout — and the airframe stayed on the ground. Stall at ~58% PWM against
85% available. Conclusion: **commanded thrust ≠ delivered thrust**, i.e. the §4
thrust model is wrong. Root cause not yet isolated.

### (b) ~0.87 Hz limit cycle [MEASURED]
Sustained physical up-down oscillation, kinematically self-consistent
(`p_z ≈ v_z / 2πf`). Ruled out by measurement: velocity-blend discretisation,
supply-voltage sag, estimator noise, LiPo cutout.

Leading hypothesis: **loop delay**. The position loop's natural frequency is
0.36 Hz (√k_p_z), so a 0.87 Hz oscillation is not simple P/D behaviour. The
`/Odometry` publish rate was never measured — **if it's ~10 Hz, that is very
likely your oscillation source.**

**Direct ask:** if your sim can report its own end-to-end pose latency and
publish rate, and can inject configurable extra latency, that would be the most
useful single feature for settling (b).

---

## 6. Conventions summary

| | |
|---|---|
| world frame | z up; poses and velocities from SLAM are in it |
| body frame | gyro and accel are in it |
| angles | radians throughout; RPY is **ZYX** from `quat` |
| length / velocity | metres, m/s |
| force / torque | newtons, N·m |
| quaternion | `qw, qx, qy, qz` (Hamilton, world←body) — note w first |
| internal time | `steady_clock`; **only** the UDP packet uses wall clock |

---

## 7. State of the flight-side code

The hardware layer is being **rewritten from scratch** — the previous one is
considered untrustworthy, which is why so much above is marked SUSPECT.

Rewrite is at `qdrone2_fc/` (formerly `qdrone_bare/`), layers L0–L2 written:
card lifecycle, DShot/arm sequence, raw sensor reads. Layers L3–L5 (estimator,
mixer, PID controller) are deliberately unwritten until bench measurements land.

**Nothing in the rewrite changes the §1 UDP contract.** Build against that and
you are insulated from the rewrite entirely.

**Currently unmeasured, so don't calibrate a sim to it:** true thrust curve,
supply-voltage ADC scale, physical motor layout, IMU accel scale error, achieved
loop rate, end-to-end latency. Bench session is imminent; these numbers will
exist shortly and are worth asking for before you tune anything.
