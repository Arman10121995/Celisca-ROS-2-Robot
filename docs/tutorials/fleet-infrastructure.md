# Physical fleet infrastructure

Open **Fleet & Swarm**, choose **fleet_facility**, enable per-member AMCL/Nav2,
review members and Run. The facility includes two sliding doors, a physical
three-metre lift/car door, a shaft/upper landing and a named dock. Its L0
occupancy is derived from the authored walls; the shaft is occupied and moving
doors remain real live obstacles.

Connect starts neutral. In the **Facility** subpage, choose a door and
Open or Close if clear. Closing waits for fresh fleet-body sweep observations;
missing peers hold the measured position instead of starting an unrequested
opening. Lift requests close the car door before moving. The car door moves
with the platform and opens only near a source floor with low measured lift
speed. Fleet wheel inputs are held neutral for members in a transitioning car;
stale infrastructure feedback also holds wheels. ALL STOP cancels infrastructure
intent and docking and holds bounded measured joint targets.

These are native force PIDs with source force/travel/velocity limits, not
animated poses. Door/lift states come from actual joint feedback. Platform
motion and a robot's complete multi-floor transport/navigation mission are
different acceptance cases. L1 planning/localization maps and RMF graph/floor
annotations still need integration before claiming automatic floor transport.

Choose one member and **Navigate selected member to dock**. The adapter
dispatches actual Nav2 and requires independent body position, heading, linear
and angular speed to settle for one simulated second. A sent command or
action acknowledgement is not docking. This is pose berthing, not a connector,
contact or charging certificate. RMF's docking callback uses a unique
request identity and completes only on that observed result; old dock results
cannot acknowledge a later task.

With the installed RMF message packages the adapter supports `/door_requests`,
`/door_states`, `/lift_requests` and `/lift_states`. Lift requests retain session
ownership, and simultaneous door requesters retain separate open requests.
Actual Nav2 corridor paths also request the main door. Typed infrastructure
transport availability does not qualify a full RMF conflict/transport mission.

The **modeled battery** field stays **unknown** for the current source bases:
their actuator-power observations have not been qualified. The estimate
integrator and dock-conditioned charging hooks exist, with explicit capacity,
hotel load, efficiency and charging parameters in `infrastructure.yaml`.
Do not set `effort_observation_verified` merely to display percentages. First
implement/measure the backend's actual motor effort/rate readback, then bind
the source receipt and model parameters. Hardware battery telemetry remains
separate, and RMF's nominal cost objects do not become measurements.

Agents reproduce assets with `scripts/generate_fleet_facility.py`. Infrastructure
launches automatically only for a map declaring `fleet_infrastructure`; existing
worlds retain their existing controllers/parameters. Other backends have static
geometry inspection, with moving fixture adapters still pending.

[Current physical cases and limits](../status/amphibious-facility-2026-10-30.md).
