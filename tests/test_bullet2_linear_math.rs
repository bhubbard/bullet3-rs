use approx::assert_relative_eq;
use bullet3::prelude::*;
use std::f32::consts::PI;

#[test]
fn test_v3triple_product() {
    let a = Vector3::new(1.0, 2.0, 3.0);
    let b = Vector3::new(4.0, 5.0, 6.0);
    let c = Vector3::new(7.0, 8.0, 10.0);

    // a . (b x c)
    let triple = a.triple(b, c);
    let cross_bc = b.cross(c);
    let dot_a = a.dot(cross_bc);

    assert_relative_eq!(triple, dot_a, epsilon = 1e-5);
    assert_relative_eq!(triple, -3.0, epsilon = 1e-5);
}

#[test]
fn test_v3angle_and_rotate() {
    let x = Vector3::X;
    let y = Vector3::Y;
    let angle = x.angle(y);
    assert_relative_eq!(angle, PI * 0.5, epsilon = 1e-5);

    // Rotate X 90 degrees around Z -> should become Y
    let rotated = x.rotate(Vector3::Z, PI * 0.5);
    assert_relative_eq!(rotated.x, 0.0, epsilon = 1e-5);
    assert_relative_eq!(rotated.y, 1.0, epsilon = 1e-5);
    assert_relative_eq!(rotated.z, 0.0, epsilon = 1e-5);
}

#[test]
fn test_3x3_transpose_multiplications() {
    let m1 = Matrix3x3::from_rows(
        Vector3::new(1.0, 2.0, 3.0),
        Vector3::new(4.0, 5.0, 6.0),
        Vector3::new(7.0, 8.0, 9.0),
    );
    let m2 = Matrix3x3::from_rows(
        Vector3::new(2.0, 0.0, 1.0),
        Vector3::new(1.0, 3.0, 2.0),
        Vector3::new(0.0, 1.0, 4.0),
    );

    // times_transpose = m1 * m2^T
    let res1 = m1.times_transpose(&m2);
    let expected1 = m1 * m2.transpose();
    for i in 0..3 {
        for j in 0..3 {
            assert_relative_eq!(res1.m[i][j], expected1.m[i][j], epsilon = 1e-5);
        }
    }

    // transpose_times = m1^T * m2
    let res2 = m1.transpose_times(&m2);
    let expected2 = m1.transpose() * m2;
    for i in 0..3 {
        for j in 0..3 {
            assert_relative_eq!(res2.m[i][j], expected2.m[i][j], epsilon = 1e-5);
        }
    }

    // mul_transpose_vector = m1^T * v
    let v = Vector3::new(1.0, 2.0, 3.0);
    let vt = m1.mul_transpose_vector(v);
    let expected_vt = m1.transpose() * v;
    assert_relative_eq!(vt.x, expected_vt.x, epsilon = 1e-5);
    assert_relative_eq!(vt.y, expected_vt.y, epsilon = 1e-5);
    assert_relative_eq!(vt.z, expected_vt.z, epsilon = 1e-5);
}

#[test]
fn test_default_motion_state() {
    let start_tf = Transform::from_translation(Vector3::new(10.0, 20.0, 30.0));
    let mut ms = DefaultMotionState::new(start_tf);

    assert_eq!(ms.get_world_transform(), start_tf);

    // Simulate physics engine updating the transform
    let new_tf = Transform::from_translation(Vector3::new(12.0, 25.0, 32.0));
    ms.set_world_transform(new_tf);
    assert_eq!(ms.get_world_transform(), new_tf);
    assert_eq!(ms.graphics_world_transform, new_tf);
}
