# Actors with real collision (R6.2, 2026-11-15)

Claude. Implemented, experimental.

## The problem

Gazebo Fortress actors are visual only. Rendering sensors (cameras, GPU
lidar) see them, but robots drive through people and traffic. No gz-sim
actor-collision system exists. The open-source ActorCollisionsPlugin is for
Gazebo Classic. On the server, an actor's pose component is only its
trajectory origin; the renderer animates it.

## The change

**`robot_lab::ActorColliders`** (`robot_lab_maps/systems/actor_colliders.cc`)
moves a companion body `<actor>__collider` to each actor's scripted pose
every step, and sets its velocity from the trajectory.

The pose is computed as ign-gazebo6's `SceneManager` animates the actor:

- one `common::TrajectoryInfo` per SDF trajectory, built from the waypoints and tension;
- the trajectories sequenced after `delay_start`;
- looping, or held at the last pose.

The collider therefore follows the drawn skin. It stays upright at its
authored height and follows x, y and heading. World systems are configured
before actors exist as entities, so the actors are found on the first update.

**`actor_collisions:=true`** is a `simulated_robot.launch.py` and
`gazebo.launch.py` argument, off by default. With it, `world_with_actor_colliders`
copies the world and adds one collider per waypoint-scripted actor:

| Actor | Collider | Mass |
|---|---|---:|
| Vehicles (`vehicle_box.dae` skin, `vehicle*`/`car*` names) | 4.4 × 1.8 × 1.5 m box | 1500 kg |
| People (walking skins, `person*`/`walker*` names) | 0.25 m × 1.7 m cylinder | 80 kg |
| Anything else | 0.3 m × 1.0 m cylinder | 50 kg |

Each collider has gravity off and no visual. Classic-format
`<animation><script><control>` actors are skipped; they don't move in
Fortress either.

## Measured

`scripts/gazebo_actor_collision_check.py`: a person actor walks 6 m along
y = 0.05 through a free 0.3 m, 2 kg box at the origin, headless, 12 s
([result.json](result.json)).

| Run | Box displacement |
|---|---:|
| without colliders | 0.000 m |
| with colliders | 3.41 m, carried along the walk to x = 3.41 |

## Limits

- **Not yet run:** a robot touching an actor, and the lidar/camera view
  during contact.
- **Approximate shapes:** the cylinder and box stand in for the skins, which
  are not animated limb by limb.
- **Off by default:** existing actor-world evidence, such as drones avoiding
  walkers in `aerial_moving`, was measured without colliders.
