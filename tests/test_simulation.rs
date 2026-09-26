use approx::assert_relative_eq;
use bullet3::prelude::*;

#[test]
fn test_freefall_kinematics() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -10.0, 0.0));

    let shape = Shape::Sphere(SphereShape::new(0.5));
    let start_tf = Transform::from_translation(Vector3::new(0.0, 100.0, 0.0));
    // Damping = 0 to test exact analytical kinematic equation: y(t) = y0 - 0.5 * g * t^2
    let info = RigidBodyConstructionInfo::new(1.0, shape, start_tf).with_damping(0.0, 0.0);
    let id = world.add_rigid_body(RigidBody::new(info));

    let dt = 1.0 / 100.0;
    let total_steps = 100; // 1.0 second

    for _ in 0..total_steps {
        world.step_simulation(dt, 1, dt);
    }

    let body = world.get_rigid_body(id).unwrap();
    // After 1 sec: v = -10 m/s, y = 100 - 0.5 * 10 * 1^2 = 95.0 m
    assert_relative_eq!(body.linear_velocity.y, -10.0, epsilon = 0.1);
    assert_relative_eq!(body.transform.origin.y, 95.0, epsilon = 0.2);
}

#[test]
fn test_box_settling_on_ground() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Ground plane at y = 0
    let ground_shape = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        ground_shape,
        Transform::IDENTITY,
    )));

    // Box of half extents 0.5 at y = 3.0 (bottom is at y = 2.5)
    let box_shape = Shape::Box(BoxShape::new(Vector3::splat(0.5)));
    let box_tf = Transform::from_translation(Vector3::new(0.0, 3.0, 0.0));
    let box_info = RigidBodyConstructionInfo::new(1.0, box_shape, box_tf)
        .with_restitution(0.0)
        .with_friction(0.8);
    let box_id = world.add_rigid_body(RigidBody::new(box_info));

    let dt = 1.0 / 60.0;
    for _ in 0..120 {
        world.step_simulation(dt, 10, dt);
    }

    let settled = world.get_rigid_body(box_id).unwrap();
    // Expected center Y is 0.5 (box half-height above plane)
    assert_relative_eq!(settled.transform.origin.y, 0.5, epsilon = 0.05);
    assert_relative_eq!(settled.linear_velocity.y, 0.0, epsilon = 0.1);
}

#[test]
fn test_elastic_momentum_conservation() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::ZERO);

    // Ball 1 at x = -2 moving at +4 m/s
    let shape1 = Shape::Sphere(SphereShape::new(0.5));
    let mut body1 = RigidBody::new(
        RigidBodyConstructionInfo::new(
            1.0,
            shape1,
            Transform::from_translation(Vector3::new(-2.0, 0.0, 0.0)),
        )
        .with_restitution(1.0)
        .with_damping(0.0, 0.0),
    );
    body1.linear_velocity = Vector3::new(4.0, 0.0, 0.0);
    let id1 = world.add_rigid_body(body1);

    // Ball 2 at x = 0 stationary
    let shape2 = Shape::Sphere(SphereShape::new(0.5));
    let body2 = RigidBody::new(
        RigidBodyConstructionInfo::new(
            1.0,
            shape2,
            Transform::from_translation(Vector3::ZERO),
        )
        .with_restitution(1.0)
        .with_damping(0.0, 0.0),
    );
    let id2 = world.add_rigid_body(body2);

    let dt = 1.0 / 60.0;
    // Run until after impact
    for _ in 0..60 {
        world.step_simulation(dt, 10, dt);
    }

    let b1 = world.get_rigid_body(id1).unwrap();
    let b2 = world.get_rigid_body(id2).unwrap();

    let total_momentum = b1.linear_velocity.x * b1.mass + b2.linear_velocity.x * b2.mass;
    assert_relative_eq!(total_momentum, 4.0, epsilon = 0.1);
    assert!(b2.linear_velocity.x > 3.0); // Momentum transferred to ball 2
}

#[test]
fn test_point_to_point_joint_distance() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Static anchor at (0, 10, 0)
    let anchor_shape = Shape::Sphere(SphereShape::new(0.1));
    let anchor_id = world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        anchor_shape,
        Transform::from_translation(Vector3::new(0.0, 10.0, 0.0)),
    )));

    // Bob at (2, 10, 0)
    let bob_shape = Shape::Sphere(SphereShape::new(0.2));
    let bob_id = world.add_rigid_body(RigidBody::new(
        RigidBodyConstructionInfo::new(
            1.0,
            bob_shape,
            Transform::from_translation(Vector3::new(2.0, 10.0, 0.0)),
        )
        .with_damping(0.0, 0.0),
    ));

    // Point-to-point constraint of length 2.0
    let constraint = Point2PointConstraint::new(
        anchor_id,
        bob_id,
        Vector3::ZERO,
        Vector3::new(-2.0, 0.0, 0.0),
    );
    world.add_constraint(constraint);

    let dt = 1.0 / 120.0;
    for _ in 0..240 {
        world.step_simulation(dt, 10, dt);
        let bob = world.get_rigid_body(bob_id).unwrap();
        let dist = bob.transform.origin.distance(Vector3::new(0.0, 10.0, 0.0));
        // The rod length should remain approximately 2.0 throughout oscillation
        assert_relative_eq!(dist, 2.0, epsilon = 0.08);
    }
}
