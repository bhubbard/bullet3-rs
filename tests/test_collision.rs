use approx::assert_relative_eq;
use bullet3::collision::narrowphase::detect_collision;
use bullet3::prelude::*;

#[test]
fn test_sphere_sphere_collision() {
    let shape_a = Shape::Sphere(SphereShape::new(1.0));
    let tf_a = Transform::from_translation(Vector3::new(0.0, 0.0, 0.0));

    let shape_b = Shape::Sphere(SphereShape::new(1.0));
    let tf_b = Transform::from_translation(Vector3::new(1.5, 0.0, 0.0));

    let manifold = detect_collision(0, &shape_a, &tf_a, 1, &shape_b, &tf_b).expect("Should collide");
    assert_eq!(manifold.points.len(), 1);

    let pt = &manifold.points[0];
    assert_relative_eq!(pt.distance, -0.5, epsilon = 1e-4); // 1.5 - 2.0 = -0.5 penetration
    assert_relative_eq!(pt.normal_world_on_b.x, -1.0, epsilon = 1e-4); // Points from B to A (-X)
}

#[test]
fn test_sphere_plane_collision() {
    let sphere = Shape::Sphere(SphereShape::new(0.5));
    let sphere_tf = Transform::from_translation(Vector3::new(0.0, 0.3, 0.0));

    let plane = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    let plane_tf = Transform::IDENTITY;

    let manifold = detect_collision(0, &sphere, &sphere_tf, 1, &plane, &plane_tf).expect("Should collide");
    assert_eq!(manifold.points.len(), 1);

    let pt = &manifold.points[0];
    assert_relative_eq!(pt.distance, -0.2, epsilon = 1e-4); // 0.3 - 0.5 = -0.2
    assert_relative_eq!(pt.normal_world_on_b.y, 1.0, epsilon = 1e-4);
}

#[test]
fn test_box_plane_collision() {
    let box_shape = Shape::Box(BoxShape::new(Vector3::new(1.0, 1.0, 1.0)));
    let box_tf = Transform::from_translation(Vector3::new(0.0, 0.8, 0.0));

    let plane = Shape::StaticPlane(StaticPlaneShape::new(Vector3::Y, 0.0));
    let plane_tf = Transform::IDENTITY;

    let manifold = detect_collision(0, &box_shape, &box_tf, 1, &plane, &plane_tf).expect("Should collide");
    assert_eq!(manifold.points.len(), 4); // 4 bottom corners penetrating y = 0

    for pt in &manifold.points {
        assert_relative_eq!(pt.distance, -0.2, epsilon = 1e-4);
        assert_relative_eq!(pt.normal_world_on_b.y, 1.0, epsilon = 1e-4);
    }
}

#[test]
fn test_dbvt_broadphase_dynamic_updates() {
    let mut dbvt = DbvtBroadphase::new();

    let aabb1 = Aabb::new(Vector3::ZERO, Vector3::new(2.0, 2.0, 2.0));
    let aabb2 = Aabb::new(Vector3::new(1.0, 0.0, 0.0), Vector3::new(3.0, 2.0, 2.0));
    let aabb3 = Aabb::new(Vector3::new(10.0, 0.0, 0.0), Vector3::new(12.0, 2.0, 2.0));

    dbvt.insert(0, aabb1);
    dbvt.insert(1, aabb2);
    dbvt.insert(2, aabb3);

    let pairs = dbvt.compute_pairs();
    assert_eq!(pairs.len(), 1);
    assert_eq!(pairs[0], BroadphasePair::new(0, 1));

    // Move body 2 to collide with body 1
    let aabb3_moved = Aabb::new(Vector3::new(1.5, 0.0, 0.0), Vector3::new(3.5, 2.0, 2.0));
    dbvt.update(2, aabb3_moved);

    let pairs_after = dbvt.compute_pairs();
    assert!(pairs_after.contains(&BroadphasePair::new(0, 1)));
    assert!(pairs_after.contains(&BroadphasePair::new(0, 2)));
    assert!(pairs_after.contains(&BroadphasePair::new(1, 2)));

    // Remove body 0
    dbvt.remove(0);
    let pairs_removed = dbvt.compute_pairs();
    assert_eq!(pairs_removed.len(), 1);
    assert_eq!(pairs_removed[0], BroadphasePair::new(1, 2));
}

#[test]
fn test_ray_casting() {
    let mut world = CollisionWorld::new();

    let sphere = Shape::Sphere(SphereShape::new(1.0));
    let tf = Transform::from_translation(Vector3::new(0.0, 0.0, 5.0));
    world.add_collision_object(CollisionObject::new(0, sphere, tf));

    let ray_from = Vector3::new(0.0, 0.0, 0.0);
    let ray_to = Vector3::new(0.0, 0.0, 10.0);

    let hit = world.ray_test(ray_from, ray_to).expect("Ray should hit sphere");
    assert_eq!(hit.body_id, 0);
    assert_relative_eq!(hit.hit_fraction, 0.4, epsilon = 1e-4); // Sphere surface is at z = 4 (4 / 10 = 0.4)
    assert_relative_eq!(hit.hit_point_world.z, 4.0, epsilon = 1e-4);
    assert_relative_eq!(hit.hit_normal_world.z, -1.0, epsilon = 1e-4);
}
