use crate::dynamics::rigid_body::RigidBody;
use crate::linear_math::{Matrix3x3, Vector3};

/// Common trait for all multi-body constraints.
/// Equivalent to Bullet's `btTypedConstraint`.
pub trait Constraint: Send + Sync {
    fn body_a(&self) -> usize;
    fn body_b(&self) -> usize;
    fn prepare(&mut self, bodies: &[Option<RigidBody>], dt: f32);
    fn solve_velocity(&mut self, bodies: &mut [Option<RigidBody>]);
    fn solve_position(&mut self, bodies: &mut [Option<RigidBody>]);
}

/// Point-to-Point (ball and socket) joint constraint.
/// Constrains pivot on Body A to pivot on Body B (`btPoint2PointConstraint`).
pub struct Point2PointConstraint {
    pub body_a: usize,
    pub body_b: usize,
    pub pivot_in_a: Vector3,
    pub pivot_in_b: Vector3,

    // Internal solver caches
    r_a: Vector3,
    r_b: Vector3,
    k_inv: Matrix3x3,
    accumulated_impulse: Vector3,
    bias: Vector3,
}

impl Point2PointConstraint {
    pub fn new(body_a: usize, body_b: usize, pivot_in_a: Vector3, pivot_in_b: Vector3) -> Self {
        Self {
            body_a,
            body_b,
            pivot_in_a,
            pivot_in_b,
            r_a: Vector3::ZERO,
            r_b: Vector3::ZERO,
            k_inv: Matrix3x3::IDENTITY,
            accumulated_impulse: Vector3::ZERO,
            bias: Vector3::ZERO,
        }
    }
}

impl Constraint for Point2PointConstraint {
    fn body_a(&self) -> usize {
        self.body_a
    }

    fn body_b(&self) -> usize {
        self.body_b
    }

    fn prepare(&mut self, bodies: &[Option<RigidBody>], dt: f32) {
        let (a, b) = match (bodies.get(self.body_a), bodies.get(self.body_b)) {
            (Some(Some(a)), Some(Some(b))) => (a, b),
            _ => return,
        };

        self.r_a = a.transform.transform_vector(self.pivot_in_a);
        self.r_b = b.transform.transform_vector(self.pivot_in_b);

        let p_a = a.transform.origin + self.r_a;
        let p_b = b.transform.origin + self.r_b;

        // Position error bias (Baumgarte stabilization)
        let error = p_a - p_b;
        let baumgarte = 0.2;
        self.bias = error * (baumgarte / dt);

        // Compute effective mass matrix K = (m_a^-1 + m_b^-1)*I - [ra]_x * I_a^-1 * [ra]_x - [rb]_x * I_b^-1 * [rb]_x
        let inv_mass = (a.inv_mass + b.inv_mass) * Matrix3x3::IDENTITY;
        let ra_skew = Matrix3x3::skew_symmetric(self.r_a);
        let rb_skew = Matrix3x3::skew_symmetric(self.r_b);

        let k = inv_mass
            - (ra_skew * a.inv_inertia_world * ra_skew)
            - (rb_skew * b.inv_inertia_world * rb_skew);

        self.k_inv = k.inverse();
    }

    fn solve_velocity(&mut self, bodies: &mut [Option<RigidBody>]) {
        let (v_a, w_a, inv_m_a, i_inv_a, v_b, w_b, inv_m_b, i_inv_b) = match (
            bodies.get(self.body_a),
            bodies.get(self.body_b),
        ) {
            (Some(Some(a)), Some(Some(b))) => (
                a.linear_velocity,
                a.angular_velocity,
                a.inv_mass,
                a.inv_inertia_world,
                b.linear_velocity,
                b.angular_velocity,
                b.inv_mass,
                b.inv_inertia_world,
            ),
            _ => return,
        };

        let vel_a = v_a + w_a.cross(self.r_a);
        let vel_b = v_b + w_b.cross(self.r_b);
        let rel_vel = vel_a - vel_b;

        let delta_v = -(rel_vel + self.bias);
        let impulse = self.k_inv * delta_v;
        self.accumulated_impulse += impulse;

        if let Some(Some(a)) = bodies.get_mut(self.body_a) {
            a.linear_velocity += impulse * inv_m_a;
            a.angular_velocity += i_inv_a * self.r_a.cross(impulse);
        }
        if let Some(Some(b)) = bodies.get_mut(self.body_b) {
            b.linear_velocity -= impulse * inv_m_b;
            b.angular_velocity -= i_inv_b * self.r_b.cross(impulse);
        }
    }

    fn solve_position(&mut self, _bodies: &mut [Option<RigidBody>]) {}
}

/// Fixed or spring distance constraint between two bodies (`btDistanceConstraint`).
pub struct DistanceConstraint {
    pub body_a: usize,
    pub body_b: usize,
    pub pivot_in_a: Vector3,
    pub pivot_in_b: Vector3,
    pub target_distance: f32,

    u: Vector3,
    effective_mass: f32,
    bias: f32,
}

impl DistanceConstraint {
    pub fn new(
        body_a: usize,
        body_b: usize,
        pivot_in_a: Vector3,
        pivot_in_b: Vector3,
        target_distance: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_in_a,
            pivot_in_b,
            target_distance,
            u: Vector3::Y,
            effective_mass: 1.0,
            bias: 0.0,
        }
    }
}

impl Constraint for DistanceConstraint {
    fn body_a(&self) -> usize {
        self.body_a
    }

    fn body_b(&self) -> usize {
        self.body_b
    }

    fn prepare(&mut self, bodies: &[Option<RigidBody>], dt: f32) {
        let (a, b) = match (bodies.get(self.body_a), bodies.get(self.body_b)) {
            (Some(Some(a)), Some(Some(b))) => (a, b),
            _ => return,
        };

        let p_a = a.transform.transform_point(self.pivot_in_a);
        let p_b = b.transform.transform_point(self.pivot_in_b);
        let d = p_a - p_b;
        let current_dist = d.length();

        if current_dist > 1e-6 {
            self.u = d / current_dist;
        } else {
            self.u = Vector3::Y;
        }

        let r_a = a.transform.transform_vector(self.pivot_in_a);
        let r_b = b.transform.transform_vector(self.pivot_in_b);

        let k = a.inv_mass
            + b.inv_mass
            + self.u.dot((a.inv_inertia_world * r_a.cross(self.u)).cross(r_a))
            + self.u.dot((b.inv_inertia_world * r_b.cross(self.u)).cross(r_b));

        self.effective_mass = if k > 1e-8 { 1.0 / k } else { 0.0 };

        let error = current_dist - self.target_distance;
        self.bias = (0.2 / dt) * error;
    }

    fn solve_velocity(&mut self, bodies: &mut [Option<RigidBody>]) {
        let (v_a, w_a, inv_m_a, i_inv_a, r_a, v_b, w_b, inv_m_b, i_inv_b, r_b) = match (
            bodies.get(self.body_a),
            bodies.get(self.body_b),
        ) {
            (Some(Some(a)), Some(Some(b))) => {
                let ra = a.transform.transform_vector(self.pivot_in_a);
                let rb = b.transform.transform_vector(self.pivot_in_b);
                (
                    a.linear_velocity,
                    a.angular_velocity,
                    a.inv_mass,
                    a.inv_inertia_world,
                    ra,
                    b.linear_velocity,
                    b.angular_velocity,
                    b.inv_mass,
                    b.inv_inertia_world,
                    rb,
                )
            }
            _ => return,
        };

        let vel_a = v_a + w_a.cross(r_a);
        let vel_b = v_b + w_b.cross(r_b);
        let c_dot = (vel_a - vel_b).dot(self.u);

        let lambda = -self.effective_mass * (c_dot + self.bias);
        let impulse = self.u * lambda;

        if let Some(Some(a)) = bodies.get_mut(self.body_a) {
            a.linear_velocity += impulse * inv_m_a;
            a.angular_velocity += i_inv_a * r_a.cross(impulse);
        }
        if let Some(Some(b)) = bodies.get_mut(self.body_b) {
            b.linear_velocity -= impulse * inv_m_b;
            b.angular_velocity -= i_inv_b * r_b.cross(impulse);
        }
    }

    fn solve_position(&mut self, _bodies: &mut [Option<RigidBody>]) {}
}

/// Hinge joint constraint allowing rotation about a single axis (`btHingeConstraint`).
pub struct HingeConstraint {
    pub body_a: usize,
    pub body_b: usize,
    pub pivot_in_a: Vector3,
    pub pivot_in_b: Vector3,
    pub axis_in_a: Vector3,
    pub axis_in_b: Vector3,

    p2p: Point2PointConstraint,
}

impl HingeConstraint {
    pub fn new(
        body_a: usize,
        body_b: usize,
        pivot_in_a: Vector3,
        pivot_in_b: Vector3,
        axis_in_a: Vector3,
        axis_in_b: Vector3,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_in_a,
            pivot_in_b,
            axis_in_a: axis_in_a.normalize(),
            axis_in_b: axis_in_b.normalize(),
            p2p: Point2PointConstraint::new(body_a, body_b, pivot_in_a, pivot_in_b),
        }
    }
}

impl Constraint for HingeConstraint {
    fn body_a(&self) -> usize {
        self.body_a
    }

    fn body_b(&self) -> usize {
        self.body_b
    }

    fn prepare(&mut self, bodies: &[Option<RigidBody>], dt: f32) {
        self.p2p.prepare(bodies, dt);
    }

    fn solve_velocity(&mut self, bodies: &mut [Option<RigidBody>]) {
        self.p2p.solve_velocity(bodies);

        // Lock orthogonal angular relative velocities
        let (a, b) = match (bodies.get(self.body_a), bodies.get(self.body_b)) {
            (Some(Some(a)), Some(Some(b))) => (a, b),
            _ => return,
        };

        let axis_a_w = a.transform.transform_vector(self.axis_in_a);
        let ang_rel = a.angular_velocity - b.angular_velocity;

        // Strip the component along hinge axis
        let along = axis_a_w * ang_rel.dot(axis_a_w);
        let perp = ang_rel - along;

        let inv_i_sum = a.inv_inertia_world + b.inv_inertia_world;
        let delta_w = -(inv_i_sum.inverse() * perp);

        if let Some(Some(a)) = bodies.get_mut(self.body_a) {
            a.angular_velocity += a.inv_inertia_world * delta_w;
        }
        if let Some(Some(b)) = bodies.get_mut(self.body_b) {
            b.angular_velocity -= b.inv_inertia_world * delta_w;
        }
    }

    fn solve_position(&mut self, bodies: &mut [Option<RigidBody>]) {
        self.p2p.solve_position(bodies);
    }
}

/// Slider (prismatic) joint constraint allowing 1D translation along an axis (`btSliderConstraint`).
pub struct SliderConstraint {
    pub body_a: usize,
    pub body_b: usize,
    pub pivot_in_a: Vector3,
    pub pivot_in_b: Vector3,
    pub slider_axis_in_a: Vector3,
    pub lower_limit: f32,
    pub upper_limit: f32,

    axis_world: Vector3,
    perp1_world: Vector3,
    perp2_world: Vector3,
    bias_perp1: f32,
    bias_perp2: f32,
    bias_limit: f32,
    limit_impulse: f32,
    is_limit_active: bool,
}

impl SliderConstraint {
    pub fn new(
        body_a: usize,
        body_b: usize,
        pivot_in_a: Vector3,
        pivot_in_b: Vector3,
        slider_axis_in_a: Vector3,
        lower_limit: f32,
        upper_limit: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_in_a,
            pivot_in_b,
            slider_axis_in_a: slider_axis_in_a.normalize(),
            lower_limit,
            upper_limit,
            axis_world: Vector3::X,
            perp1_world: Vector3::Y,
            perp2_world: Vector3::Z,
            bias_perp1: 0.0,
            bias_perp2: 0.0,
            bias_limit: 0.0,
            limit_impulse: 0.0,
            is_limit_active: false,
        }
    }
}

impl Constraint for SliderConstraint {
    fn body_a(&self) -> usize {
        self.body_a
    }

    fn body_b(&self) -> usize {
        self.body_b
    }

    fn prepare(&mut self, bodies: &[Option<RigidBody>], dt: f32) {
        let (a, b) = match (bodies.get(self.body_a), bodies.get(self.body_b)) {
            (Some(Some(a)), Some(Some(b))) => (a, b),
            _ => return,
        };

        let axis = a.transform.transform_vector(self.slider_axis_in_a).normalize();
        self.axis_world = axis;

        let p1 = if axis.x.abs() > 0.57735 {
            Vector3::new(axis.y, -axis.x, 0.0).normalize()
        } else {
            Vector3::new(0.0, axis.z, -axis.y).normalize()
        };
        self.perp1_world = p1;
        self.perp2_world = axis.cross(p1).normalize();

        let pos_a = a.transform.transform_point(self.pivot_in_a);
        let pos_b = b.transform.transform_point(self.pivot_in_b);
        let diff = pos_a - pos_b;

        let proj_perp1 = diff.dot(self.perp1_world);
        let proj_perp2 = diff.dot(self.perp2_world);
        let proj_axis = diff.dot(self.axis_world);

        self.bias_perp1 = (0.2 / dt) * proj_perp1;
        self.bias_perp2 = (0.2 / dt) * proj_perp2;

        if proj_axis < self.lower_limit {
            self.is_limit_active = true;
            self.bias_limit = (0.2 / dt) * (proj_axis - self.lower_limit);
        } else if proj_axis > self.upper_limit {
            self.is_limit_active = true;
            self.bias_limit = (0.2 / dt) * (proj_axis - self.upper_limit);
        } else {
            self.is_limit_active = false;
            self.bias_limit = 0.0;
        }
    }

    fn solve_velocity(&mut self, bodies: &mut [Option<RigidBody>]) {
        let (v_a, inv_m_a, v_b, inv_m_b) = match (
            bodies.get(self.body_a),
            bodies.get(self.body_b),
        ) {
            (Some(Some(a)), Some(Some(b))) => (a.linear_velocity, a.inv_mass, b.linear_velocity, b.inv_mass),
            _ => return,
        };

        let inv_m = inv_m_a + inv_m_b;
        if inv_m < 1e-8 {
            return;
        }

        let rel_v = v_a - v_b;

        // Solve perpendicular translation lock
        for &(dir, bias) in &[
            (self.perp1_world, self.bias_perp1),
            (self.perp2_world, self.bias_perp2),
        ] {
            let lambda = -(rel_v.dot(dir) + bias) / inv_m;
            let imp = dir * lambda;
            if let Some(Some(a)) = bodies.get_mut(self.body_a) {
                a.linear_velocity += imp * inv_m_a;
            }
            if let Some(Some(b)) = bodies.get_mut(self.body_b) {
                b.linear_velocity -= imp * inv_m_b;
            }
        }

        // Solve linear limits along slider axis
        if self.is_limit_active {
            let lambda = -(rel_v.dot(self.axis_world) + self.bias_limit) / inv_m;
            self.limit_impulse += lambda;
            let imp = self.axis_world * lambda;
            if let Some(Some(a)) = bodies.get_mut(self.body_a) {
                a.linear_velocity += imp * inv_m_a;
            }
            if let Some(Some(b)) = bodies.get_mut(self.body_b) {
                b.linear_velocity -= imp * inv_m_b;
            }
        }
    }

    fn solve_position(&mut self, _bodies: &mut [Option<RigidBody>]) {}
}

/// Cone-Twist constraint for ragdoll joints, shoulders, and hips (`btConeTwistConstraint`).
pub struct ConeTwistConstraint {
    pub body_a: usize,
    pub body_b: usize,
    pub pivot_in_a: Vector3,
    pub pivot_in_b: Vector3,
    pub swing_span_1: f32,
    pub swing_span_2: f32,
    pub twist_span: f32,

    p2p: Point2PointConstraint,
}

impl ConeTwistConstraint {
    pub fn new(
        body_a: usize,
        body_b: usize,
        pivot_in_a: Vector3,
        pivot_in_b: Vector3,
        swing_span_1: f32,
        swing_span_2: f32,
        twist_span: f32,
    ) -> Self {
        Self {
            body_a,
            body_b,
            pivot_in_a,
            pivot_in_b,
            swing_span_1,
            swing_span_2,
            twist_span,
            p2p: Point2PointConstraint::new(body_a, body_b, pivot_in_a, pivot_in_b),
        }
    }
}

impl Constraint for ConeTwistConstraint {
    fn body_a(&self) -> usize {
        self.body_a
    }

    fn body_b(&self) -> usize {
        self.body_b
    }

    fn prepare(&mut self, bodies: &[Option<RigidBody>], dt: f32) {
        self.p2p.prepare(bodies, dt);
    }

    fn solve_velocity(&mut self, bodies: &mut [Option<RigidBody>]) {
        self.p2p.solve_velocity(bodies);

        // Check relative orientation cone limits
        let (a, b) = match (bodies.get(self.body_a), bodies.get(self.body_b)) {
            (Some(Some(a)), Some(Some(b))) => (a, b),
            _ => return,
        };

        let diff_q = a.transform.rotation() * b.transform.rotation().inverse();
        let (axis, angle) = diff_q.to_axis_angle();

        let max_span = self.swing_span_1.max(self.swing_span_2);
        if angle > max_span {
            let error = angle - max_span;
            let rel_ang_vel = (a.angular_velocity - b.angular_velocity).dot(axis);
            let inv_i_sum = a.inv_inertia_world + b.inv_inertia_world;
            let k = axis.dot(inv_i_sum * axis);
            if k > 1e-8 {
                let lambda = -(rel_ang_vel + 0.2 * error) / k;
                let imp = axis * lambda;
                if let Some(Some(a)) = bodies.get_mut(self.body_a) {
                    a.angular_velocity += a.inv_inertia_world * imp;
                }
                if let Some(Some(b)) = bodies.get_mut(self.body_b) {
                    b.angular_velocity -= b.inv_inertia_world * imp;
                }
            }
        }
    }

    fn solve_position(&mut self, bodies: &mut [Option<RigidBody>]) {
        self.p2p.solve_position(bodies);
    }
}
