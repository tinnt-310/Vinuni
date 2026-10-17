# FR10 force sensor handoff / commissioning gate

The force algorithm is internal to the existing motion pipeline:
input pose → conditioning/contact reference → safety → continuous IK and
collision/braking validation → MotionExecutor/Ruckig → ros2_control → ServoJ.
It never loads a competing joint controller or calls firmware FT_Control.
Physical force deployment remains locked until the actual sensor, tooling and
limits are commissioned. Docker CAD wrench is simulation evidence only.

## Fairino 3.9.3 source contract

The supported hardware plugin owns the only SDK connection. It reads the same
ROBOT_STATE_PKG as encoders and exports these diagnostic sources:

| Topic | Packet field | Meaning |
|---|---|---|
| `/fairino_driver/ft_raw` | `ft_sensor_raw_data[6]` | Sensor-native force N / torque Nm |
| `/fairino_driver/ft_reference` | `ft_sensor_data[6]` | Processed/reference data, separate from raw |
| `/fairino_driver/ft_source_status` | `ft_sensor_active`, packet freshness | Active=1, reset=0; active does not certify calibration |

This separation follows the [3.9.3 packet documentation](https://fairino-doc-en.readthedocs.io/3.9.3/SDKManual/CPPDataStructure.html)
and [force API manual](https://fairino-doc-en.readthedocs.io/3.9.3/SDKManual/CPPRobotForceControl.html).
The packet has no independently verified FT sample timestamp or overload bit.
ROS stamps represent observation of a new controller packet, not a fabricated
sensor acquisition time. A frozen sensor can require additional sensor-specific
supervision even while robot packets arrive. No overload bit is invented.

The raw frame defaults to `fairino_ft_sensor`; `ft_sensor_frame` hardware param
can set the measured mounting frame. Reference frame defaults empty until the
operator supplies `ft_reference_frame` matching cabinet RCS. Reference data is
not automatically assigned the sensor/TCP frame and is not a raw fallback.
Publishers use nonblocking realtime_tools; no additional FT RPC is made in read.
No FT_Activate, FT_SetZero, FT_SetConfig or cabinet mode/reset call is issued.

Select adapter inputs with `force.wrench_topic`, `force.source_status_topic`
and `force.require_source_status`. For raw packet commissioning use the raw
and status topics above, the measured source frame and calibrated extrinsics.
Required status rejects missing/offline/stale evidence at admission. Offline
and overload duplicates fail immediately; online duplicates cannot restore
online state or refresh freshness. A repeated wrench stamp does not refresh
the independent watchdog. Adapter `/ft_sensor/status` is derived telemetry,
never an input source status.

## Conditioning and tare

Configure the fixed sensor→TCP rotation and `p_TCP→sensor` in metres. Verify
known forces and moments, sign, mass and CoG with the attached charging tool.
Canonical wrench is tool-on-environment at TCP in fixed insertion axes.

`force.gravity_compensation.owner` is `application` (default) or `source`.
Application gravity subtraction requires calibrated mass/CoG/reference gravity.
Source ownership requires independent evidence of upstream load removal;
processed `ft_sensor_data` alone does not establish this. Enabling application
subtraction while owner=source is rejected to prevent double compensation.

Tare is residual bias **after gravity removal**, with the payload still
attached, at a stationary, collision-free preinsert clearance pose. It is not
an unloaded-tool operation and never zeros a contact wrench. WrenchTare requires
25 qualified control observations over >=192 ms source time, with every
unique raw substep included in mean AND variance, bounded sample gaps,
residual magnitude and sample variance. Invalid/gapped/moving/contact evidence
restarts the window. Adapter uses measured joint velocity; missing velocity
cannot establish stationarity. A stopped command/IK seed is insufficient.
Previous bias stays active until the replacement qualifies.
Qualifications run every control observation, including encoder-only motion
between FT packets. Such motion/offline evidence invalidates the window;
a packet first observed while moving cannot later enter tare unchanged.

`/ft_sensor/tare` queues a request in permitted idle/preinsert phases;
its acknowledgement is not calibration completion. Observe subsequent status
and FT_ZERO completion. The request does not issue a cabinet FT_SetZero.
Geometry clearance and stationary feedback must qualify independently of FT;
a small force alone cannot prove no contact. A calibration model does not make
unexpected unmodeled physical obstacles observable to software.

## Reusable algorithm API

The ROS-free C++ libraries expose:

- `WrenchCompensator::apply(raw, source_from_gravity_frame)` and `setBias`;
- `WrenchTare::observe(sample, gravity_removed, stationary_no_contact)` or
  `observeBatch(samples, gravity_removed_samples, count, stationary_no_contact)`;
- `ForcePositionController::propose(nominal, desired, measured, dt, equilibrium)`
  and `commit`, retaining the original overload without equilibrium;
- `ContactReference::observe` on every measured tick, then `propose/commit`,
  with radius/speed/source budget, a frozen
  fixed entry point transported along the last accepted orientation and a final
  loaded low-moment seating axis;
- `KeyedYawProbe::observe/propose/canCommit/commit` measures bounded yaw
  endpoints and weights their actual signed contact evidence. A physical
  wall estimates center one commissioned mechanical half-clearance inward;
  a quiet actuator envelope is neither a wall nor a center. Configure
  `KeyedYawProbeConfig::contact_free_half_span_rad` (ROS parameter
  `force.search.yaw_free_half_span_rad`) from measured mechanical clearance.
  Its default infinity means unknown clearance and retains the legacy
  observed midpoint. Simulation derives this estimate from the canonical
  pin-center radii and radial clearance; this does not calibrate real tooling. Its original
  two-argument `torque(state, yaw)` API remains available.
  `desiredTorque(state, yaw, measured_reaction, nominal_moment)` returns the
  absolute canonical desired moment: SEEK adds the locating moment; CENTERING
  cancels measured reaction in the motion objective and applies bounded
  position feedback; COMPLETE restores the nominal objective. Raw safety
  wrench remains unchanged. A fresh loaded low-moment window is required
  after COMPLETE before acquiring the final axis;
- `ContactIntervalProbe::observe/propose/canCommit/commit` is the shared ROS-free
  interval observer for XY/Newtons and yaw/radians/Nm. Contact qualification is
  distinct from valid sensor data. Configure positive XY spring, measured
  half-clearance and bounded probe loads; unknown clearance preserves midpoint.
  `ContactReference::desiredWrench` combines sequential probes with the nominal
  canonical task wrench; it never publishes joints. Wall observations remove
  nominal load and depth/rotation transport. Pending starts/boundaries/centers
  revoke on unqualified observations while the worker waits;
- `InsertionCorridor` and `observeInsertionGeometry(corridor, actual, ...)`
  keep explicit withdrawal depth/tracking/mating on one frozen measured axis.
  FT conditioning retains its immutable input frame;
- `ForceSession::update/canCommitOnlineTarget/commitOnlineTarget`, keeping all motion state on the
  accepted target transaction.

The caller must supply calibrated frames, canonical wrench, measured pose/q,
source/receipt clocks and genuine backend acceptance. Recheck `canCommitOnlineTarget` before actuator admission and commit only
after accepted publication. Intervening unqualified moment/source gaps revoke
an unissued anchor without rolling back sensor history. Never commit a pose
merely because IK succeeded. Python evidence helpers `fr10_wrench_calibration.py` and
`fr10_pose_uncertainty.py` provide independent known-load checks and bounded
perception-error injection; they are not robot actuators.

## Simulation geometry and acceptance

Current matched_socket_v2 uses a matched plug/socket CAD pair, keyed bores and
explicit shoulder contact. The historical spring-plate and mismatched
cong_sac/JLR geometry are not the matched acceptance model. Gazebo and MuJoCo
use independent physical pose/contact evidence; the evaluator always retains
the true port geometry when input pose is perturbed. Native Docker uses CAD
quasi-static contact at measured SDK q, not a physical FT sensor.

Legacy matched_socket_commissioning.yaml remains the baseline. The draft
pose_uncertainty_commissioning.yaml inherits it and adds bounded reference
search; it is not a physical deployment profile. Its acceptance status is in
`test_results/fr10_admittance_pose_error_20261003/WORK_LOG.md` and run summaries.
An acquired anchor is a mechanically seated estimate from FT/q, not proof of
true CAD center, latch state or electrical connection. Independent physical fit
must pass in simulation. HOLD freezes position and monitors force; faults use
controlled stop with no automatic reset/retract/retry. An explicit uncertainty
profile retract withdraws on the measured accepted insertion axis to clearance.
Passing the lead-in only establishes CAPTURED translation, not an orientation
certificate. ACQUIRED requires loaded seating with bounded moment dwell.
Native virtual-contact acceptance is labeled VIRTUAL_CAD_PASS/FAIL separately
from independent dynamic Gazebo/MuJoCo contact evidence.

## Required physical commissioning evidence

Record sensor model/version/mounting, source rate/latency/dropout/clip/overload
behavior, temperature bias/noise, extrinsics, mass/CoG and gravity owner. Verify
signed loads/moments in several orientations with attached payload and qualified
no-contact tare. Derive gains, limits and contact/braking envelopes from these
measurements. Verify sole SDK owner, encoder freshness and ServoJ timing without
loosening lead/singularity barriers. Commission contact-detect-and-stop before
bounded insertion, stable HOLD and explicit retract. Mounted operation also
requires fresh physical base localization and parking feedback, not wheel
odometry alone. Current simulation thresholds are not certified hardware limits.

## Gazebo source comparison, draft stage26/27

The passive contact plugin reports tool-on-environment force and moment about
the physical TCP from actual collision loads; it excludes gravity/internal
loads. `/ft_sensor/wrench` remains the payload-inclusive joint load cell.
`gazebo_contact_wrench_commissioning.yaml` explicitly selects the ideal contact
source and gravity owner `source`; it never substitutes sources automatically.
Both retain the same no-contact tare, raw safety and independent fit gates.
This source is a simulation instrument, not Fairino raw/reference FT or hardware
calibration proof. Gazebo 11 ODE feedback is stored in link frames, despite the
Contact.hh world-frame comment. ODE reverses a static-first contact into a sole
dynamic body in feedback slot1; Gazebo retains collision1's rotation for that
slot. The decoder resolves the physical slot before link→world rotation, shifts
the moment using the tool's COM and rotates the final sum to TCP. It never reads
the unwritten static-body slot or suppresses an invalid physical load.
See the producer [ODEPhysics.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEPhysics.cc)
and [dJointAttach](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/deps/opende/src/ode.cpp).
Stage27 contact-source125 us failed from unwritten-slot data; the original FAIL
is retained. Stage31 four real ODE load/frame/contact-order fixtures PASS;
full contact-source convergence/loaded comparison is pending. Stage30 MuJoCo
duplicate-FT after independently verified HOLD passes typed stale fault,
acknowledged braking and node-first shutdown. It is not real FT commissioning.
