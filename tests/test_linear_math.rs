use approx::assert_relative_eq;
use bullet3::prelude::*;
use std::f32::consts::PI;

#[test]
fn test_vector3_arithmetic() {
    let a = Vector3::new(1.0, 2.0, 3.0);
    let b = Vector3::new(4.0, 5.0, 6.0);

    assert_eq!(a + b, Vector3::new(5.0, 7.0, 9.0));
    assert_eq!(b - a, Vector3::new(3.0, 3.0, 3.0));
    assert_eq!(a * 2.0, Vector3::new(2.0, 4.0, 6.0));
    assert_eq!(b / 2.0, Vector3::new(2.0, 2.5, 3.0));

    assert_relative_eq!(a.dot(b), 32.0);
    assert_eq!(Vector3::X.cross(Vector3::Y), Vector3::Z);
    assert_eq!(Vector3::Y.cross(Vector3::Z), Vector3::X);
    assert_eq!(Vector3::Z.cross(Vector3::X), Vector3::Y);

    assert_relative_eq!(Vector3::new(3.0, 4.0, 0.0).length(), 5.0);
    assert_relative_eq!(Vector3::new(3.0, 4.0, 0.0).normalize().length(), 1.0);
}

#[test]
fn test_matrix3x3_inverse_and_determinant() {
    let m = Matrix3x3::from_rows(
        Vector3::new(1.0, 2.0, 3.0),
        Vector3::new(0.0, 1.0, 4.0),
        Vector3::new(5.0, 6.0, 0.0),
    );

    let det = m.determinant();
    assert_relative_eq!(det, 1.0, epsilon = 1e-4);

    let inv = m.inverse();
    let identity = m * inv;

    for i in 0..3 {
        for j in 0..3 {
            let expected = if i == j { 1.0 } else { 0.0 };
            assert_relative_eq!(identity.m[i][j], expected, epsilon = 1e-4);
        }
    }
}

#[test]
fn test_quaternion_rotation_and_slerp() {
    // 90 degree rotation about Z axis
    let q = Quaternion::from_axis_angle(Vector3::Z, PI * 0.5);
    let v = Vector3::X;
    let rotated = q.rotate_vector(v);

    assert_relative_eq!(rotated.x, 0.0, epsilon = 1e-5);
    assert_relative_eq!(rotated.y, 1.0, epsilon = 1e-5);
    assert_relative_eq!(rotated.z, 0.0, epsilon = 1e-5);

    // Slerp halfway (45 degrees)
    let slerped = Quaternion::IDENTITY.slerp(q, 0.5);
    let mid_rotated = slerped.rotate_vector(v);
    let expected_mid = (0.5f32).sqrt();
    assert_relative_eq!(mid_rotated.x, expected_mid, epsilon = 1e-4);
    assert_relative_eq!(mid_rotated.y, expected_mid, epsilon = 1e-4);
}

#[test]
fn test_transform_composition_and_inverse() {
    let t1 = Transform::from_translation(Vector3::new(1.0, 2.0, 3.0));
    let q = Quaternion::from_axis_angle(Vector3::Y, PI * 0.5);
    let t2 = Transform::from_rotation_quaternion(Vector3::ZERO, q);

    let combined = t1 * t2;
    let p = Vector3::new(1.0, 0.0, 0.0);
    let transformed = combined.transform_point(p);

    // Rotated p around Y: (0, 0, -1), then translated: (1, 2, 2)
    assert_relative_eq!(transformed.x, 1.0, epsilon = 1e-4);
    assert_relative_eq!(transformed.y, 2.0, epsilon = 1e-4);
    assert_relative_eq!(transformed.z, 2.0, epsilon = 1e-4);

    let inv = combined.inverse();
    let back = inv.transform_point(transformed);
    assert_relative_eq!(back.x, p.x, epsilon = 1e-4);
    assert_relative_eq!(back.y, p.y, epsilon = 1e-4);
    assert_relative_eq!(back.z, p.z, epsilon = 1e-4);
}

#[test]
fn test_aabb_operations() {
    let aabb1 = Aabb::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(2.0, 2.0, 2.0));
    let aabb2 = Aabb::new(Vector3::new(1.0, 1.0, 1.0), Vector3::new(3.0, 3.0, 3.0));
    let aabb3 = Aabb::new(Vector3::new(5.0, 5.0, 5.0), Vector3::new(6.0, 6.0, 6.0));

    assert!(aabb1.intersects(&aabb2));
    assert!(!aabb1.intersects(&aabb3));

    let merged = aabb1.merge(&aabb2);
    assert_eq!(merged.min, Vector3::new(0.0, 0.0, 0.0));
    assert_eq!(merged.max, Vector3::new(3.0, 3.0, 3.0));

    // Raycast hit
    let hit_t = aabb1.ray_test(Vector3::new(-2.0, 1.0, 1.0), Vector3::new(4.0, 1.0, 1.0));
    assert!(hit_t.is_some());
    let t = hit_t.unwrap();
    assert_relative_eq!(t, 2.0 / 6.0, epsilon = 1e-4); // Hits at x = 0
}
