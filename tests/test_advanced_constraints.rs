use approx::assert_relative_eq;
use bullet3::prelude::*;

#[test]
fn test_slider_constraint_linear_limits() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::ZERO);

    // Static body A at (0, 0, 0)
    let body_a = RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        Shape::Sphere(SphereShape::new(0.5)),
        Transform::IDENTITY,
    ));
    let id_a = world.add_rigid_body(body_a);

    // Dynamic body B at (1, 0, 0)
    let mut body_b = RigidBody::new(
        RigidBodyConstructionInfo::new(
            1.0,
            Shape::Sphere(SphereShape::new(0.5)),
            Transform::from_translation(Vector3::new(1.0, 0.0, 0.0)),
        )
        .with_damping(0.0, 0.0),
    );
    // Push body B along X with velocity = 10 m/s
    body_b.linear_velocity = Vector3::new(10.0, 0.0, 0.0);
    let id_b = world.add_rigid_body(body_b);

    // Slider along X axis with limits [-2.0, 2.0]
    let slider = SliderConstraint::new(
        id_a,
        id_b,
        Vector3::ZERO,
        Vector3::ZERO,
        Vector3::X,
        -2.0,
        2.0,
    );
    world.add_constraint(slider);

    let dt = 1.0 / 60.0;
    for _ in 0..60 {
        world.step_simulation(dt, 10, dt);
    }

    let b = world.get_rigid_body(id_b).unwrap();
    // Body B should be clamped near the upper limit of 2.0
    assert!(b.transform.origin.x <= 2.1);
    // Body B should have zero translation along Y and Z
    assert_relative_eq!(b.transform.origin.y, 0.0, epsilon = 0.05);
    assert_relative_eq!(b.transform.origin.z, 0.0, epsilon = 0.05);
}

#[test]
fn test_cone_twist_constraint_limits() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::ZERO);

    // Static body A
    let id_a = world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        Shape::Sphere(SphereShape::new(0.5)),
        Transform::IDENTITY,
    )));

    // Dynamic body B
    let mut body_b = RigidBody::new(
        RigidBodyConstructionInfo::new(
            1.0,
            Shape::Sphere(SphereShape::new(0.5)),
            Transform::from_translation(Vector3::new(0.0, 1.0, 0.0)),
        )
        .with_damping(0.0, 0.0),
    );
    body_b.angular_velocity = Vector3::new(2.0, 0.0, 0.0);
    let id_b = world.add_rigid_body(body_b);

    let cone_twist = ConeTwistConstraint::new(
        id_a,
        id_b,
        Vector3::new(0.0, 0.5, 0.0),
        Vector3::new(0.0, -0.5, 0.0),
        0.5, // 0.5 rad swing span
        0.5,
        0.5,
    );
    world.add_constraint(cone_twist);

    let dt = 1.0 / 60.0;
    for _ in 0..60 {
        world.step_simulation(dt, 10, dt);
    }

    let b = world.get_rigid_body(id_b).unwrap();
    let q = b.transform.rotation();
    let (_, angle) = q.to_axis_angle();
    // Angular rotation should stay restrained
    assert!(angle <= 1.0);
}

#[test]
fn test_distance_constraint_maintains_length() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Static anchor
    let id_a = world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        Shape::Sphere(SphereShape::new(0.1)),
        Transform::from_translation(Vector3::new(0.0, 5.0, 0.0)),
    )));

    // Dynamic bob at distance 3.0
    let id_b = world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        1.0,
        Shape::Sphere(SphereShape::new(0.2)),
        Transform::from_translation(Vector3::new(3.0, 5.0, 0.0)),
    )));

    let dist_constraint = DistanceConstraint::new(
        id_a,
        id_b,
        Vector3::ZERO,
        Vector3::ZERO,
        3.0,
    );
    world.add_constraint(dist_constraint);

    let dt = 1.0 / 60.0;
    for _ in 0..120 {
        world.step_simulation(dt, 10, dt);
        let bob = world.get_rigid_body(id_b).unwrap();
        let current_dist = bob.transform.origin.distance(Vector3::new(0.0, 5.0, 0.0));
        assert_relative_eq!(current_dist, 3.0, epsilon = 0.1);
    }
}
