use clap::{Parser, Subcommand};
use bullet3::prelude::*;
use std::time::Instant;

#[derive(Parser, Debug)]
#[command(name = "bullet3")]
#[command(author = "Brandon Hubbard")]
#[command(version = "0.1.0")]
#[command(about = "Pure Rust fork & implementation of Bullet 3 Physics SDK", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Simulate a single box falling onto a static ground plane
    Drop {
        #[arg(long, default_value_t = 10.0)]
        height: f32,
        #[arg(short, long, default_value_t = 120)]
        steps: usize,
    },
    /// Simulate a stable vertical stack of boxes (Gauss-Seidel stress test)
    Stack {
        #[arg(short, long, default_value_t = 5)]
        boxes: usize,
        #[arg(short, long, default_value_t = 180)]
        steps: usize,
    },
    /// Simulate a pendulum attached by a Point-to-Point joint
    Pendulum {
        #[arg(short, long, default_value_t = 120)]
        steps: usize,
    },
    /// Simulate elastic billiard collisions between spheres
    Billiards {
        #[arg(short, long, default_value_t = 100)]
        steps: usize,
    },
    /// Benchmark raw simulation throughput (steps per second)
    Benchmark {
        #[arg(short, long, default_value_t = 1000)]
        steps: usize,
        #[arg(short, long, default_value_t = 20)]
        bodies: usize,
    },
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Commands::Drop { height, steps } => run_drop(height, steps),
        Commands::Stack { boxes, steps } => run_stack(boxes, steps),
        Commands::Pendulum { steps } => run_pendulum(steps),
        Commands::Billiards { steps } => run_billiards(steps),
        Commands::Benchmark { steps, bodies } => run_benchmark(steps, bodies),
    }
}

fn run_drop(initial_height: f32, steps: usize) {
    println!("📦 Bullet3-RS: Dropping Box Simulation");
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Ground plane
    let ground_shape = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    let ground_info = RigidBodyConstructionInfo::new(0.0, ground_shape, Transform::IDENTITY);
    world.add_rigid_body(RigidBody::new(ground_info));

    // Dynamic box
    let box_shape = Shape::Box(BoxShape::new(Vector3::new(0.5, 0.5, 0.5)));
    let start_tf = Transform::from_translation(Vector3::new(0.0, initial_height, 0.0));
    let box_info = RigidBodyConstructionInfo::new(1.0, box_shape, start_tf)
        .with_restitution(0.4)
        .with_friction(0.6);
    let box_id = world.add_rigid_body(RigidBody::new(box_info));

    println!("Initial State: Height = {:.2}m, Velocity = 0.0 m/s", initial_height);
    println!("Step |   Time   |   Height (Y)   |   Velocity Y   | Status");
    println!("-----+----------+----------------+----------------+---------");

    let dt = 1.0 / 60.0;
    for i in 0..steps {
        world.step_simulation(dt, 10, dt);
        let body = world.get_rigid_body(box_id).unwrap();

        if i % 10 == 0 || i == steps - 1 {
            let status = if body.transform.origin.y <= 0.52 {
                "Grounded"
            } else {
                "Freefall"
            };
            println!(
                "{:4} | {:6.2}s  |    {:6.3}m     |    {:6.3}m/s   | {}",
                i,
                i as f32 * dt,
                body.transform.origin.y,
                body.linear_velocity.y,
                status
            );
        }
    }
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

fn run_stack(num_boxes: usize, steps: usize) {
    println!("🧱 Bullet3-RS: Box Stacking Stability Demo");
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Ground plane
    let ground_shape = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        ground_shape,
        Transform::IDENTITY,
    )));

    let mut box_ids = Vec::new();
    let box_half = 0.5;

    for i in 0..num_boxes {
        let y_pos = box_half + (i as f32 * box_half * 2.05);
        let shape = Shape::Box(BoxShape::new(Vector3::splat(box_half)));
        let tf = Transform::from_translation(Vector3::new(0.0, y_pos, 0.0));
        let info = RigidBodyConstructionInfo::new(1.0, shape, tf)
            .with_friction(0.8)
            .with_damping(0.05, 0.1);
        box_ids.push(world.add_rigid_body(RigidBody::new(info)));
    }

    println!("Stacked {} boxes vertically. Simulating {} steps...", num_boxes, steps);
    let dt = 1.0 / 60.0;
    for _ in 0..steps {
        world.step_simulation(dt, 10, dt);
    }

    println!("Final box positions after settling:");
    for (idx, &id) in box_ids.iter().enumerate() {
        let b = world.get_rigid_body(id).unwrap();
        println!(
            "  Box #{}: Y = {:.3}m, Vel = ({:.3}, {:.3}, {:.3})",
            idx, b.transform.origin.y, b.linear_velocity.x, b.linear_velocity.y, b.linear_velocity.z
        );
    }
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

fn run_pendulum(steps: usize) {
    println!("🎯 Bullet3-RS: Joint Constraint (Pendulum) Demo");
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Static anchor at (0, 5, 0)
    let anchor_shape = Shape::Sphere(SphereShape::new(0.1));
    let anchor_tf = Transform::from_translation(Vector3::new(0.0, 5.0, 0.0));
    let anchor_id = world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        anchor_shape,
        anchor_tf,
    )));

    // Bob at (3, 5, 0) - suspended horizontally
    let bob_shape = Shape::Sphere(SphereShape::new(0.3));
    let bob_tf = Transform::from_translation(Vector3::new(3.0, 5.0, 0.0));
    let bob_id = world.add_rigid_body(RigidBody::new(
        RigidBodyConstructionInfo::new(2.0, bob_shape, bob_tf).with_damping(0.01, 0.01),
    ));

    // Point to Point Constraint connecting anchor to bob
    let constraint = Point2PointConstraint::new(
        anchor_id,
        bob_id,
        Vector3::ZERO,
        Vector3::new(-3.0, 0.0, 0.0),
    );
    world.add_constraint(constraint);

    println!("Pendulum released from horizontal (X = 3.0, Y = 5.0)");
    let dt = 1.0 / 60.0;
    for i in 0..steps {
        world.step_simulation(dt, 10, dt);
        if i % 15 == 0 {
            let bob = world.get_rigid_body(bob_id).unwrap();
            let dist = bob.transform.origin.distance(Vector3::new(0.0, 5.0, 0.0));
            println!(
                "Step {:3} | Time: {:4.2}s | Bob Pos: ({:5.2}, {:5.2}, {:5.2}) | Rod Length: {:.3}m",
                i,
                i as f32 * dt,
                bob.transform.origin.x,
                bob.transform.origin.y,
                bob.transform.origin.z,
                dist
            );
        }
    }
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

fn run_billiards(steps: usize) {
    println!("🎱 Bullet3-RS: Elastic Collision Demo");
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::ZERO);

    // Cue ball moving at +5 m/s along X
    let shape_a = Shape::Sphere(SphereShape::new(0.5));
    let tf_a = Transform::from_translation(Vector3::new(-3.0, 0.0, 0.0));
    let mut ball_a = RigidBody::new(
        RigidBodyConstructionInfo::new(1.0, shape_a, tf_a)
            .with_restitution(1.0)
            .with_damping(0.0, 0.0),
    );
    ball_a.linear_velocity = Vector3::new(5.0, 0.0, 0.0);
    let id_a = world.add_rigid_body(ball_a);

    // Target ball stationary at (0, 0, 0)
    let shape_b = Shape::Sphere(SphereShape::new(0.5));
    let tf_b = Transform::from_translation(Vector3::ZERO);
    let ball_b = RigidBody::new(
        RigidBodyConstructionInfo::new(1.0, shape_b, tf_b)
            .with_restitution(1.0)
            .with_damping(0.0, 0.0),
    );
    let id_b = world.add_rigid_body(ball_b);

    let dt = 1.0 / 60.0;
    for i in 0..steps {
        world.step_simulation(dt, 10, dt);
        if i % 10 == 0 {
            let a = world.get_rigid_body(id_a).unwrap();
            let b = world.get_rigid_body(id_b).unwrap();
            println!(
                "Step {:3} | Ball A (x={:5.2}, vx={:5.2}) | Ball B (x={:5.2}, vx={:5.2})",
                i,
                a.transform.origin.x,
                a.linear_velocity.x,
                b.transform.origin.x,
                b.linear_velocity.x
            );
        }
    }
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

fn run_benchmark(steps: usize, num_bodies: usize) {
    println!("⚡ Bullet3-RS: Performance Benchmark");
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    println!("Config: {} bodies, {} simulation steps", num_bodies, steps);

    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Ground
    let ground_shape = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        ground_shape,
        Transform::IDENTITY,
    )));

    // Spawn falling spheres
    for i in 0..num_bodies {
        let x = (i % 5) as f32 * 1.5 - 3.0;
        let z = (i / 5) as f32 * 1.5 - 3.0;
        let y = 5.0 + (i as f32 * 0.8);
        let shape = Shape::Sphere(SphereShape::new(0.4));
        let tf = Transform::from_translation(Vector3::new(x, y, z));
        let body = RigidBody::new(RigidBodyConstructionInfo::new(1.0, shape, tf).with_restitution(0.3));
        world.add_rigid_body(body);
    }

    let dt = 1.0 / 60.0;
    let start = Instant::now();

    for _ in 0..steps {
        world.step_simulation(dt, 10, dt);
    }

    let duration = start.elapsed();
    let secs = duration.as_secs_f64();
    let hz = steps as f64 / secs;

    println!("Completed {} steps in {:.3} seconds ({:.1} physics steps/second)", steps, secs, hz);
    println!("Average time per step: {:.2} µs", (secs / steps as f64) * 1_000_000.0);
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}
