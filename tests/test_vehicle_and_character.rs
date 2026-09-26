use approx::assert_relative_eq;
use bullet3::prelude::*;

#[test]
fn test_raycast_vehicle_simulation() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Ground plane at y = 0
    let ground_shape = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        ground_shape,
        Transform::IDENTITY,
    )));

    // Chassis box (half extents 1.0, 0.5, 2.0) placed at y = 2.0
    let chassis_shape = Shape::Box(BoxShape::new(Vector3::new(1.0, 0.5, 2.0)));
    let chassis_tf = Transform::from_translation(Vector3::new(0.0, 2.0, 0.0));
    let chassis_id = world.add_rigid_body(RigidBody::new(
        RigidBodyConstructionInfo::new(800.0, chassis_shape, chassis_tf)
            .with_damping(0.1, 0.1),
    ));

    let mut vehicle = RaycastVehicle::new(chassis_id);

    // 4 wheels (front-left, front-right, rear-left, rear-right)
    let wheel_dir = Vector3::new(0.0, -1.0, 0.0);
    let wheel_axle = Vector3::new(-1.0, 0.0, 0.0);
    let suspension_rest = 0.6;
    let wheel_radius = 0.35;

    vehicle.add_wheel(Vector3::new(-1.1, -0.2, 1.4), wheel_dir, wheel_axle, suspension_rest, wheel_radius);
    vehicle.add_wheel(Vector3::new(1.1, -0.2, 1.4), wheel_dir, wheel_axle, suspension_rest, wheel_radius);
    vehicle.add_wheel(Vector3::new(-1.1, -0.2, -1.4), wheel_dir, wheel_axle, suspension_rest, wheel_radius);
    vehicle.add_wheel(Vector3::new(1.1, -0.2, -1.4), wheel_dir, wheel_axle, suspension_rest, wheel_radius);

    // Steer front wheels 15 degrees (0.26 rad)
    vehicle.set_steering_value(0, 0.26);
    vehicle.set_steering_value(1, 0.26);
    assert_eq!(vehicle.wheels[0].steering, 0.26);

    // Apply engine force to rear wheels
    vehicle.apply_engine_force(2, 500.0);
    vehicle.apply_engine_force(3, 500.0);

    // Simulate for 60 frames (1 second)
    let dt = 1.0 / 60.0;
    for _ in 0..60 {
        vehicle.update_vehicle(&mut world, dt);
        world.step_simulation(dt, 10, dt);
    }

    // Vehicle chassis should stay supported above ground by suspension
    let chassis = world.get_rigid_body(chassis_id).unwrap();
    assert!(chassis.transform.origin.y > 0.4);
    assert!(chassis.transform.origin.y < 2.5);
}

#[test]
fn test_kinematic_character_controller_movement() {
    let mut world = DiscreteDynamicsWorld::new().with_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Ground plane at y = 0
    let ground_shape = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    world.add_rigid_body(RigidBody::new(RigidBodyConstructionInfo::new(
        0.0,
        ground_shape,
        Transform::IDENTITY,
    )));

    // Character capsule at (0, 1.0, 0)
    let char_shape = Shape::Capsule(CapsuleShape::new(0.4, 1.8));
    let char_tf = Transform::from_translation(Vector3::new(0.0, 1.0, 0.0));
    let char_id = world.add_rigid_body(RigidBody::new(
        RigidBodyConstructionInfo::new(0.0, char_shape, char_tf), // Kinematic/ghost body
    ));

    let mut controller = KinematicCharacterController::new(char_id, 0.35);

    // Set forward walk direction (+Z at 5 m/s)
    controller.set_walk_direction(Vector3::new(0.0, 0.0, 5.0));

    let dt = 1.0 / 60.0;
    for _ in 0..60 {
        controller.update_action(&mut world, dt);
        world.step_simulation(dt, 10, dt);
    }

    let char_body = world.get_rigid_body(char_id).unwrap();
    // Character should have walked forward in +Z direction
    assert!(char_body.transform.origin.z > 4.5);
    // Character should be grounded on the plane (capsule center at y = 1.3, feet at y = 0.0)
    assert_relative_eq!(char_body.transform.origin.y, 1.3, epsilon = 0.1);
}
