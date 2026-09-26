use crate::collision::narrowphase::ContactManifold;
use crate::dynamics::constraint::Constraint;
use crate::dynamics::rigid_body::RigidBody;
use crate::linear_math::Vector3;

/// Contact constraint row solved during sequential impulse iterations.
#[derive(Debug, Clone)]
struct ContactConstraintRow {
    normal: Vector3,
    tangent_1: Vector3,
    tangent_2: Vector3,
    rel_pos_a: Vector3,
    rel_pos_b: Vector3,
    normal_effective_mass: f32,
    tangent_1_effective_mass: f32,
    tangent_2_effective_mass: f32,
    restitution_bias: f32,
    penetration_bias: f32,
    friction: f32,
    accumulated_normal_impulse: f32,
    accumulated_friction_1: f32,
    accumulated_friction_2: f32,
}

/// Sequential Impulse Constraint Solver.
/// Direct equivalent to Bullet's `btSequentialImpulseConstraintSolver`.
pub struct SequentialImpulseConstraintSolver {
    pub num_iterations: usize,
    pub baumgarte: f32,
    pub slop: f32,
}

impl SequentialImpulseConstraintSolver {
    pub fn new() -> Self {
        Self {
            num_iterations: 10,
            baumgarte: 0.2,
            slop: 0.005,
        }
    }

    pub fn solve(
        &mut self,
        bodies: &mut [Option<RigidBody>],
        manifolds: &mut [ContactManifold],
        constraints: &mut [Box<dyn Constraint>],
        dt: f32,
    ) {
        if dt <= 1e-6 {
            return;
        }

        // 1. Prepare constraints
        for c in constraints.iter_mut() {
            c.prepare(bodies, dt);
        }

        // 2. Prepare contact constraints
        let mut manifold_rows: Vec<Vec<ContactConstraintRow>> = Vec::with_capacity(manifolds.len());

        for manifold in manifolds.iter_mut() {
            let (body_a, body_b) = match (bodies.get(manifold.body_a), bodies.get(manifold.body_b)) {
                (Some(Some(a)), Some(Some(b))) => (a, b),
                _ => {
                    manifold_rows.push(Vec::new());
                    continue;
                }
            };

            let combined_friction = (body_a.friction * body_b.friction).sqrt();
            let combined_restitution = body_a.restitution.max(body_b.restitution);

            let mut rows = Vec::with_capacity(manifold.points.len());

            for pt in &mut manifold.points {
                let r_a = pt.position_world_a - body_a.transform.origin;
                let r_b = pt.position_world_b - body_b.transform.origin;
                let normal = pt.normal_world_on_b; // Points from B to A

                // Construct orthonormal tangent vectors for friction
                let tangent_1 = if normal.x.abs() > 0.57735 {
                    Vector3::new(normal.y, -normal.x, 0.0).normalize()
                } else {
                    Vector3::new(0.0, normal.z, -normal.y).normalize()
                };
                let tangent_2 = normal.cross(tangent_1).normalize();

                // Compute effective masses
                let k_norm = compute_effective_mass(body_a, body_b, r_a, r_b, normal);
                let k_tan1 = compute_effective_mass(body_a, body_b, r_a, r_b, tangent_1);
                let k_tan2 = compute_effective_mass(body_a, body_b, r_a, r_b, tangent_2);

                let v_a = body_a.velocity_at_point(r_a);
                let v_b = body_b.velocity_at_point(r_b);
                let rel_v = v_a - v_b;
                let normal_vel = rel_v.dot(normal);

                // Restitution bias (velocity threshold to prevent micro-bouncing)
                let restitution_bias = if normal_vel < -0.5 && combined_restitution > 0.0 {
                    -combined_restitution * normal_vel
                } else {
                    0.0
                };

                // Baumgarte stabilization penetration bias
                let penetration = (pt.distance + self.slop).min(0.0);
                let penetration_bias = -(self.baumgarte / dt) * penetration;

                // Warm start impulses
                let acc_norm = pt.applied_impulse;
                let acc_f1 = pt.applied_friction_1;
                let acc_f2 = pt.applied_friction_2;

                rows.push(ContactConstraintRow {
                    normal,
                    tangent_1,
                    tangent_2,
                    rel_pos_a: r_a,
                    rel_pos_b: r_b,
                    normal_effective_mass: k_norm,
                    tangent_1_effective_mass: k_tan1,
                    tangent_2_effective_mass: k_tan2,
                    restitution_bias,
                    penetration_bias,
                    friction: combined_friction,
                    accumulated_normal_impulse: acc_norm,
                    accumulated_friction_1: acc_f1,
                    accumulated_friction_2: acc_f2,
                });
            }

            manifold_rows.push(rows);
        }

        // 3. Warm-starting: apply cached impulses from previous step
        for (m_idx, manifold) in manifolds.iter().enumerate() {
            let rows = &manifold_rows[m_idx];
            for row in rows {
                let warm_impulse = row.normal * row.accumulated_normal_impulse
                    + row.tangent_1 * row.accumulated_friction_1
                    + row.tangent_2 * row.accumulated_friction_2;

                apply_impulse(bodies, manifold.body_a, manifold.body_b, warm_impulse, row.rel_pos_a, row.rel_pos_b);
            }
        }

        // 4. Sequential Impulse Iterations
        for _iter in 0..self.num_iterations {
            // Solve joints
            for c in constraints.iter_mut() {
                c.solve_velocity(bodies);
            }

            // Solve contact constraints
            for (m_idx, manifold) in manifolds.iter().enumerate() {
                let rows = &mut manifold_rows[m_idx];
                for row in rows.iter_mut() {
                    // --- A. Normal Non-Penetration Constraint ---
                    let (v_a, v_b) = {
                        let a = bodies[manifold.body_a].as_ref().unwrap();
                        let b = bodies[manifold.body_b].as_ref().unwrap();
                        (a.velocity_at_point(row.rel_pos_a), b.velocity_at_point(row.rel_pos_b))
                    };
                    let rel_vel = v_a - v_b;
                    let c_dot = rel_vel.dot(row.normal);

                    let delta_impulse = -(c_dot - row.restitution_bias - row.penetration_bias) * row.normal_effective_mass;
                    let old_impulse = row.accumulated_normal_impulse;
                    row.accumulated_normal_impulse = (old_impulse + delta_impulse).max(0.0);
                    let actual_delta = row.accumulated_normal_impulse - old_impulse;

                    apply_impulse(
                        bodies,
                        manifold.body_a,
                        manifold.body_b,
                        row.normal * actual_delta,
                        row.rel_pos_a,
                        row.rel_pos_b,
                    );

                    // --- B. Tangent Friction Constraints ---
                    let max_friction = row.friction * row.accumulated_normal_impulse;

                    // Friction 1
                    let (v_a, v_b) = {
                        let a = bodies[manifold.body_a].as_ref().unwrap();
                        let b = bodies[manifold.body_b].as_ref().unwrap();
                        (a.velocity_at_point(row.rel_pos_a), b.velocity_at_point(row.rel_pos_b))
                    };
                    let rel_vel = v_a - v_b;
                    let f1_dot = rel_vel.dot(row.tangent_1);
                    let delta_f1 = -f1_dot * row.tangent_1_effective_mass;
                    let old_f1 = row.accumulated_friction_1;
                    row.accumulated_friction_1 = (old_f1 + delta_f1).clamp(-max_friction, max_friction);
                    let actual_f1 = row.accumulated_friction_1 - old_f1;

                    apply_impulse(
                        bodies,
                        manifold.body_a,
                        manifold.body_b,
                        row.tangent_1 * actual_f1,
                        row.rel_pos_a,
                        row.rel_pos_b,
                    );

                    // Friction 2
                    let (v_a, v_b) = {
                        let a = bodies[manifold.body_a].as_ref().unwrap();
                        let b = bodies[manifold.body_b].as_ref().unwrap();
                        (a.velocity_at_point(row.rel_pos_a), b.velocity_at_point(row.rel_pos_b))
                    };
                    let rel_vel = v_a - v_b;
                    let f2_dot = rel_vel.dot(row.tangent_2);
                    let delta_f2 = -f2_dot * row.tangent_2_effective_mass;
                    let old_f2 = row.accumulated_friction_2;
                    row.accumulated_friction_2 = (old_f2 + delta_f2).clamp(-max_friction, max_friction);
                    let actual_f2 = row.accumulated_friction_2 - old_f2;

                    apply_impulse(
                        bodies,
                        manifold.body_a,
                        manifold.body_b,
                        row.tangent_2 * actual_f2,
                        row.rel_pos_a,
                        row.rel_pos_b,
                    );
                }
            }
        }

        // Store solved impulses back into manifold for warm starting next step
        for (m_idx, manifold) in manifolds.iter_mut().enumerate() {
            let rows = &manifold_rows[m_idx];
            for (pt_idx, pt) in manifold.points.iter_mut().enumerate() {
                if let Some(row) = rows.get(pt_idx) {
                    pt.applied_impulse = row.accumulated_normal_impulse;
                    pt.applied_friction_1 = row.accumulated_friction_1;
                    pt.applied_friction_2 = row.accumulated_friction_2;
                }
            }
        }
    }
}

impl Default for SequentialImpulseConstraintSolver {
    fn default() -> Self {
        Self::new()
    }
}

fn compute_effective_mass(
    body_a: &RigidBody,
    body_b: &RigidBody,
    r_a: Vector3,
    r_b: Vector3,
    dir: Vector3,
) -> f32 {
    let rn_a = r_a.cross(dir);
    let rn_b = r_b.cross(dir);

    let k = body_a.inv_mass
        + body_b.inv_mass
        + dir.dot((body_a.inv_inertia_world * rn_a).cross(r_a))
        + dir.dot((body_b.inv_inertia_world * rn_b).cross(r_b));

    if k > 1e-8 {
        1.0 / k
    } else {
        0.0
    }
}

fn apply_impulse(
    bodies: &mut [Option<RigidBody>],
    body_a: usize,
    body_b: usize,
    impulse: Vector3,
    r_a: Vector3,
    r_b: Vector3,
) {
    if let Some(Some(a)) = bodies.get_mut(body_a) {
        a.apply_impulse(impulse, r_a);
    }
    if let Some(Some(b)) = bodies.get_mut(body_b) {
        b.apply_impulse(-impulse, r_b);
    }
}
