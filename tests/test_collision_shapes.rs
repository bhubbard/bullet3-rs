use approx::assert_relative_eq;
use bullet3::prelude::*;

#[test]
fn test_cone_shape_properties() {
    let cone = ConeShape::new(1.0, 2.0);
    let inertia = cone.calculate_local_inertia(10.0);

    // Iy = 3/10 * m * r^2 = 0.3 * 10 * 1 = 3.0
    // Ix = Iz = 3/80 * m * (4*r^2 + h^2) = 3/80 * 10 * (4 + 4) = 3/80 * 80 = 3.0
    assert_relative_eq!(inertia.y, 3.0, epsilon = 1e-4);
    assert_relative_eq!(inertia.x, 3.0, epsilon = 1e-4);
    assert_relative_eq!(inertia.z, 3.0, epsilon = 1e-4);

    // Support point along +Y should point toward apex at y = +1.0
    let sup_up = cone.local_supporting_vertex(Vector3::Y);
    assert!(sup_up.y >= 1.0);

    // Support point along -Y should point toward base disk at y = -1.0
    let sup_down = cone.local_supporting_vertex(-Vector3::Y);
    assert!(sup_down.y <= -1.0);
}

#[test]
fn test_capsule_shape_properties() {
    let capsule = CapsuleShape::new(0.5, 2.0); // radius 0.5, height 2.0
    let aabb = capsule.calculate_local_aabb();

    assert!(aabb.min.y <= -1.5);
    assert!(aabb.max.y >= 1.5);
    assert!(aabb.min.x <= -0.5);
    assert!(aabb.max.x >= 0.5);

    let sup_x = capsule.local_supporting_vertex(Vector3::X);
    assert!(sup_x.x >= 0.5);
}

#[test]
fn test_cylinder_shape_properties() {
    let cylinder = CylinderShape::new(Vector3::new(1.0, 2.0, 1.0));
    let inertia = cylinder.calculate_local_inertia(12.0);

    // Iy = 0.5 * m * r^2 = 0.5 * 12 * 1 = 6.0
    assert_relative_eq!(inertia.y, 6.0, epsilon = 1e-4);

    let sup_y = cylinder.local_supporting_vertex(Vector3::Y);
    assert_relative_eq!(sup_y.y, 2.0, epsilon = 1e-4);
}

#[test]
fn test_convex_hull_shape_properties() {
    let vertices = vec![
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(-1.0, 1.0, 1.0),
        Vector3::new(1.0, -1.0, 1.0),
        Vector3::new(-1.0, -1.0, 1.0),
        Vector3::new(1.0, 1.0, -1.0),
        Vector3::new(-1.0, 1.0, -1.0),
        Vector3::new(1.0, -1.0, -1.0),
        Vector3::new(-1.0, -1.0, -1.0),
    ];
    let hull = ConvexHullShape::new(vertices);
    let aabb = hull.calculate_local_aabb();

    assert!(aabb.min.x <= -1.0);
    assert!(aabb.max.x >= 1.0);

    let sup = hull.local_supporting_vertex(Vector3::new(1.0, 1.0, 1.0));
    assert!(sup.x >= 1.0 && sup.y >= 1.0 && sup.z >= 1.0);
}

#[test]
fn test_compound_shape_properties() {
    let mut compound = CompoundShape::new();
    let s1 = Shape::Sphere(SphereShape::new(0.5));
    let s2 = Shape::Sphere(SphereShape::new(0.5));

    compound.add_child_shape(Transform::from_translation(Vector3::new(-1.0, 0.0, 0.0)), s1);
    compound.add_child_shape(Transform::from_translation(Vector3::new(1.0, 0.0, 0.0)), s2);

    let aabb = compound.calculate_local_aabb();
    assert!(aabb.min.x <= -1.5);
    assert!(aabb.max.x >= 1.5);

    let inertia = compound.calculate_local_inertia(2.0);
    assert!(inertia.y > 0.0);
}

#[test]
fn test_ghost_object_sensor_tracking() {
    let mut ghost = GhostObject::new(
        100,
        Shape::Sphere(SphereShape::new(2.0)),
        Transform::from_translation(Vector3::new(0.0, 0.0, 0.0)),
    );

    // Manually register an overlapping body (as simulated during broadphase / trigger events)
    ghost.overlapping_objects.push(1);
    ghost.overlapping_objects.push(2);

    assert_eq!(ghost.overlapping_objects.len(), 2);
    assert!(ghost.overlapping_objects.contains(&1));
    assert!(ghost.overlapping_objects.contains(&2));
}
