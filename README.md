# bullet3-rs 🦀🎯

A high-performance, deterministic 3D rigid body dynamics and collision detection physics engine written in pure, idiomatic Rust.

Forked and ported from the industry-standard [Bullet 3 Physics SDK](https://github.com/bulletphysics/bullet3) by Erwin Coumans.

[![CI](https://github.com/bhubbard/bullet3-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/bhubbard/bullet3-rs/actions)
[![License: Zlib/MIT/Apache-2.0](https://img.shields.io/badge/license-Zlib%20%7C%20MIT%20%7C%20Apache--2.0-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-2021%20edition-orange.svg)](https://www.rust-lang.org)

---

## ⚡ Why bullet3-rs?

Bullet Physics is one of the most battle-tested physics engines in history, powering Blender, commercial visual effects, robotics research (PyBullet), and AAA video games.

`bullet3-rs` brings the complete core architecture of Bullet into the modern Rust ecosystem:
- **Zero C / C++ FFI**: Pure 100% safe, memory-safe Rust with zero external C++ toolchains or build scripts required.
- **Blazingly Fast**: Executes over **15,000+ physics simulation steps per second** in single-threaded benchmark workloads.
- **Deterministic Fixed Sub-Stepping**: Exactly reproduces Bullet's time-accumulator substepping for frame-rate-independent simulation consistency.
- **Projected Gauss-Seidel Solver**: Sequential impulse constraint solver with warm starting, Baumgarte error reduction stabilization, and Coulomb friction cone constraints.
- **Modern Ergonomics**: Idiomatic Rust builder patterns, operator overloading, optional Serde serialization, and clean modular APIs.

---

## 🏛 Architecture Mapping: Bullet3 C++ vs bullet3-rs

| Bullet 3 C++ Module | `bullet3-rs` Equivalent | Description |
| :--- | :--- | :--- |
| `LinearMath` | [`bullet3::linear_math`](src/linear_math/) | `Vector3`, `Matrix3x3`, `Quaternion`, `Transform`, `Aabb` |
| `btDbvtBroadphase` | [`bullet3::collision::DbvtBroadphase`](src/collision/broadphase.rs) | Dynamic Bounding Volume Tree (AABB tree) with logarithmic pair queries |
| `btCollisionShape` | [`bullet3::collision::Shape`](src/collision/shapes.rs) | Box, Sphere, Capsule, Cylinder, StaticPlane, ConvexHull, Compound |
| `btCollisionWorld` | [`bullet3::collision::CollisionWorld`](src/collision/dispatch.rs) | Ray-casting queries, broadphase tracking, and discrete collision dispatch |
| Narrowphase | [`bullet3::collision::narrowphase`](src/collision/narrowphase.rs) | Specialized SAT (Box-Box), analytic tests, and GJK convex polytope solver |
| `btRigidBody` | [`bullet3::dynamics::RigidBody`](src/dynamics/rigid_body.rs) | Mass, world inertia tensors, velocity damping, forces, and sleeping |
| `btSequentialImpulseConstraintSolver` | [`bullet3::dynamics::SequentialImpulseConstraintSolver`](src/dynamics/solver.rs) | Iterative impulse solver with contact restitution, friction, & warm starting |
| `btTypedConstraint` | [`bullet3::dynamics::Constraint`](src/dynamics/constraint.rs) | `Point2PointConstraint`, `HingeConstraint`, `DistanceConstraint` |
| `btDiscreteDynamicsWorld` | [`bullet3::dynamics::DiscreteDynamicsWorld`](src/dynamics/world.rs) | Master simulation pipeline: forces → broadphase → narrowphase → solver → integrate |

---

## 🚀 Quick Start

Add `bullet3-rs` to your `Cargo.toml`:

```toml
[dependencies]
bullet3 = { package = "bullet3-rs", git = "https://github.com/bhubbard/bullet3-rs.git" }
```

### Dropping Box on Static Ground

```rust
use bullet3::prelude::*;

fn main() {
    // 1. Create dynamics world with standard gravity (9.81 m/s² downwards)
    let mut world = DiscreteDynamicsWorld::new()
        .with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // 2. Add static ground plane at y = 0
    let ground_shape = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    let ground_info = RigidBodyConstructionInfo::new(0.0, ground_shape, Transform::IDENTITY);
    world.add_rigid_body(RigidBody::new(ground_info));

    // 3. Add falling box at height y = 10.0
    let box_shape = Shape::Box(BoxShape::new(Vector3::splat(0.5)));
    let box_tf = Transform::from_translation(Vector3::new(0.0, 10.0, 0.0));
    let box_info = RigidBodyConstructionInfo::new(1.0, box_shape, box_tf)
        .with_restitution(0.3)
        .with_friction(0.6);
    let box_id = world.add_rigid_body(RigidBody::new(box_info));

    // 4. Step simulation for 60 ticks (1.0 second at 60 Hz)
    let dt = 1.0 / 60.0;
    for _ in 0..60 {
        world.step_simulation(dt, 10, dt);
    }

    let box_body = world.get_rigid_body(box_id).unwrap();
    println!("Settled position: Y = {:.3}m", box_body.transform.origin.y);
}
```

---

## 🎮 CLI Demos & Benchmarking

`bullet3-rs` includes a built-in CLI for inspecting simulations, verifying constraint stability, and running performance benchmarks.

### 1. Dropping Box Simulation
```bash
cargo run -- drop --height 10.0 --steps 60
```

### 2. Multi-Box Stacking Stability Test
Stress-tests the Sequential Impulse Constraint Solver by stacking rigid bodies vertically:
```bash
cargo run -- stack --boxes 5 --steps 120
```

### 3. Joint Constraint Pendulum
Simulates a spherical bob attached to a static anchor via `Point2PointConstraint`:
```bash
cargo run -- pendulum --steps 90
```

### 4. Elastic Sphere Billiard Collisions
Verifies momentum conservation in elastic multi-sphere collisions:
```bash
cargo run -- billiards --steps 60
```

### 5. High-Throughput Performance Benchmark
```bash
cargo run -- benchmark --steps 1000 --bodies 20
```

---

## 🔬 Testing & Verification

The test suite covers linear algebra, geometric narrowphase routines, constraint dynamics, and ray-casting:

```bash
cargo test
```

- `test_linear_math`: Vector, matrix inverse, quaternion slerp, transform composition, and AABB slab intersection.
- `test_collision`: Sphere-Sphere, Sphere-Plane, Box-Plane, SAT Box-Box, and DBVT broadphase pair tracking.
- `test_simulation`: Freefall kinematics ($y = y_0 - \frac{1}{2}gt^2$), ground settling, momentum conservation, and joint distance constraints.

---

## 📄 License

Licensed under the [Zlib License](LICENSE) (matching original Bullet Physics), or optionally under [MIT](LICENSE) / [Apache-2.0](LICENSE).

### Acknowledgments

- **Erwin Coumans** and the original [Bullet Physics](https://github.com/bulletphysics/bullet3) contributors for pioneering open-source real-time physics simulation.
