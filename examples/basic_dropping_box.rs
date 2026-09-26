//! Bullet 3 Basic Dropping Box Example
//!
//! Replicating the canonical `BasicExample.cpp` from the original Bullet Physics SDK.

use bullet3::prelude::*;

fn main() {
    println!("=== Bullet3-RS Basic Example ===");

    // 1. Create discrete dynamics world with standard gravity
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // 2. Add ground plane at y = 0
    let ground_shape = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    let ground_body = RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        ground_shape,
        Transform::IDENTITY,
    ));
    world.add_rigid_body(ground_body);

    // 3. Add falling box at y = 10.0
    let box_shape = Shape::Box(BoxShape::new(Vector3::splat(1.0)));
    let box_tf = Transform::from_translation(Vector3::new(0.0, 10.0, 0.0));
    let box_body = RigidBody::new(
        RigidBodyConstructionInfo::new(1.0, box_shape, box_tf)
            .with_restitution(0.3)
            .with_friction(0.5),
    );
    let box_id = world.add_rigid_body(box_body);

    // 4. Step simulation for 100 frames at 60 Hz
    let dt = 1.0 / 60.0;
    for step in 0..100 {
        world.step_simulation(dt, 10, dt);

        if step % 10 == 0 {
            let body = world.get_rigid_body(box_id).unwrap();
            println!(
                "Step {:3} | Time: {:.2}s | Pos Y: {:.3}m | Vel Y: {:.3}m/s",
                step,
                step as f32 * dt,
                body.transform.origin.y,
                body.linear_velocity.y
            );
        }
    }

    let final_body = world.get_rigid_body(box_id).unwrap();
    println!("Final box position: Y = {:.3}m", final_body.transform.origin.y);
    println!("Simulation finished successfully!");
}
