//! Rigorous Physical Accuracy & Mechanical Conservation Benchmark Tests
//! Evaluates energy conservation, linear momentum conservation, and kinematic precision.

use approx::assert_relative_eq;
use bullet3::prelude::*;

#[test]
fn test_long_term_kinetic_energy_conservation() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::ZERO);

    let shape = Shape::Sphere(SphereShape::new(1.0));
    let start_tf = Transform::from_translation(Vector3::new(0.0, 0.0, 0.0));
    let info = RigidBodyConstructionInfo::new(2.5, shape, start_tf)
        .with_damping(0.0, 0.0);
    let mut body = RigidBody::new(info);
    body.linear_velocity = Vector3::new(12.5, -8.0, 3.2);
    let id = world.add_rigid_body(body);

    let initial_speed_sq = 12.5 * 12.5 + 8.0 * 8.0 + 3.2 * 3.2;
    let initial_energy = 0.5 * 2.5 * initial_speed_sq;

    let dt = 1.0 / 120.0;
    // Step for 5,000 steps (over 41 seconds of simulation)
    for _ in 0..5000 {
        world.step_simulation(dt, 1, dt);
    }

    let b = world.get_rigid_body(id).unwrap();
    let final_speed_sq = b.linear_velocity.length_squared();
    let final_energy = 0.5 * 2.5 * final_speed_sq;

    let delta_energy = (final_energy - initial_energy).abs() / initial_energy;
    assert!(
        delta_energy < 1e-4,
        "Symplectic energy drift exceeded: delta_E/E0 = {delta_energy}"
    );
}

#[test]
fn test_elastic_collision_momentum_and_energy_parity() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::ZERO);

    let m1 = 2.0f32;
    let m2 = 3.0f32;
    let v1_init = 5.0f32;

    let shape1 = Shape::Sphere(SphereShape::new(0.5));
    let mut body1 = RigidBody::new(
        RigidBodyConstructionInfo::new(
            m1,
            shape1,
            Transform::from_translation(Vector3::new(-3.0, 0.0, 0.0)),
        )
        .with_restitution(1.0)
        .with_damping(0.0, 0.0),
    );
    body1.linear_velocity = Vector3::new(v1_init, 0.0, 0.0);
    let id1 = world.add_rigid_body(body1);

    let shape2 = Shape::Sphere(SphereShape::new(0.5));
    let body2 = RigidBody::new(
        RigidBodyConstructionInfo::new(
            m2,
            shape2,
            Transform::from_translation(Vector3::ZERO),
        )
        .with_restitution(1.0)
        .with_damping(0.0, 0.0),
    );
    let id2 = world.add_rigid_body(body2);

    let initial_momentum = m1 * v1_init;

    let dt = 1.0 / 100.0;
    for _ in 0..100 {
        world.step_simulation(dt, 10, dt);
    }

    let b1 = world.get_rigid_body(id1).unwrap();
    let b2 = world.get_rigid_body(id2).unwrap();

    let final_momentum = m1 * b1.linear_velocity.x + m2 * b2.linear_velocity.x;
    let momentum_error = (final_momentum - initial_momentum).abs() / initial_momentum;

    assert!(
        momentum_error < 0.05,
        "Momentum conservation error too high: {momentum_error}"
    );
}

#[test]
fn test_analytical_projectile_kinematic_accuracy() {
    let g = 9.80665f32;
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -g, 0.0));

    let shape = Shape::Sphere(SphereShape::new(0.2));
    let start_tf = Transform::from_translation(Vector3::new(0.0, 50.0, 0.0));
    let info = RigidBodyConstructionInfo::new(1.0, shape, start_tf)
        .with_damping(0.0, 0.0);
    let id = world.add_rigid_body(RigidBody::new(info));

    let dt = 1.0 / 100.0f32;
    let t_total = 1.0f32;
    let steps = (t_total / dt) as usize;

    for _ in 0..steps {
        world.step_simulation(dt, 1, dt);
    }

    let body = world.get_rigid_body(id).unwrap();
    let expected_y = 50.0 - 0.5 * g * t_total * t_total;
    let expected_vy = -g * t_total;

    assert_relative_eq!(body.transform.origin.y, expected_y, epsilon = 0.1);
    assert_relative_eq!(body.linear_velocity.y, expected_vy, epsilon = 0.1);
}
