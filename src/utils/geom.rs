use crate::utils::primitives::{EPS, Vector2D, Vector3D};

/// Represents the orientation of three points in 3D space.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    C,   // Collinear
    CW,  // Clockwise
    CCW, // Counterclockwise
}

/// Represents the type of intersection between line segments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntersectionType {
    Proper,
    Endpoint,
}

#[must_use]
pub fn calculate_triangle_area(t: (Vector3D, Vector3D, Vector3D)) -> f64 {
    (t.1 - t.0).cross(&(t.2 - t.0)).magnitude() * 0.5
}

#[must_use]
pub fn are_points_coplanar(a: Vector3D, b: Vector3D, c: Vector3D, d: Vector3D) -> bool {
    (b - a).cross(&(c - a)).dot(&(d - a)) == 0.
}

#[must_use]
pub fn calculate_orientation(a: Vector3D, b: Vector3D, c: Vector3D, n: Vector3D) -> Orientation {
    let orientation = (b - a).cross(&(c - a)).dot(&n);
    if orientation > 0. {
        Orientation::CCW
    } else if orientation < 0. {
        Orientation::CW
    } else {
        Orientation::C
    }
}

#[must_use]
pub fn calculate_clockwise_angle(a: Vector3D, b: Vector3D, c: Vector3D, n: Vector3D) -> f64 {
    let ab = (b - a).normalize();
    let ac = (c - a).normalize();
    let angle = ab.angle(&ac);
    if calculate_orientation(a, b, c, n) == Orientation::CCW {
        2.0f64.mul_add(std::f64::consts::PI, -angle)
    } else {
        angle
    }
}

#[must_use]
pub fn project_point_onto_plane(point: Vector3D, plane: (Vector3D, Vector3D), reference: Vector3D) -> Vector2D {
    Vector2D::new((point - reference).dot(&plane.0), (point - reference).dot(&plane.1))
}

#[must_use]
pub fn is_point_inside_triangle(p: Vector3D, t: (Vector3D, Vector3D, Vector3D)) -> bool {
    let s1 = calculate_triangle_area((t.0, t.1, p));
    let s2 = calculate_triangle_area((t.1, t.2, p));
    let s3 = calculate_triangle_area((t.2, t.0, p));
    let st = calculate_triangle_area(t);
    (s1 + s2 + s3 - st).abs() < EPS && (0.0 - EPS..=st + EPS).contains(&s1) && (0.0 - EPS..=st + EPS).contains(&s2) && (0.0 - EPS..=st + EPS).contains(&s3)
}

#[must_use]
pub fn is_within_inclusive_range(a: f64, b: f64, c: f64) -> bool {
    if b < c { (b..=c).contains(&a) } else { (c..=b).contains(&a) }
}

#[must_use]
pub fn calculate_2d_lineseg_intersection(p_u: Vector2D, p_v: Vector2D, q_u: Vector2D, q_v: Vector2D) -> Option<(Vector2D, IntersectionType)> {
    let (x1, x2, x3, x4, y1, y2, y3, y4) = (p_u.x, p_v.x, q_u.x, q_v.x, p_u.y, p_v.y, q_u.y, q_v.y);

    let t_numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4);
    let u_numerator = (x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2);
    let denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    if denominator.abs() < EPS {
        return None;
    }

    if is_within_inclusive_range(t_numerator, 0.0, denominator) {
        let t = t_numerator / denominator;
        if t.abs() < EPS {
            return Some((p_u, IntersectionType::Endpoint));
        }
        if (t - 1.0).abs() < EPS {
            return Some((p_v, IntersectionType::Endpoint));
        }
        let sx_t = t.mul_add(x2 - x1, x1);
        let sy_t = t.mul_add(y2 - y1, y1);
        let s_t = Vector2D::new(sx_t, sy_t);

        Some((s_t, IntersectionType::Proper))
    } else if is_within_inclusive_range(u_numerator, 0.0, denominator) {
        let u = u_numerator / denominator;
        if u.abs() < EPS {
            return Some((q_u, IntersectionType::Endpoint));
        }
        if (u - 1.0).abs() < EPS {
            return Some((q_v, IntersectionType::Endpoint));
        }
        let sx_u = u.mul_add(x4 - x3, x3);
        let sy_u = u.mul_add(y4 - y3, y3);
        let s_u = Vector2D::new(sx_u, sy_u);

        Some((s_u, IntersectionType::Proper))
    } else {
        None
    }
}

#[must_use]
pub fn calculate_3d_lineseg_intersection(p_u: Vector3D, p_v: Vector3D, q_u: Vector3D, q_v: Vector3D) -> Option<(Vector3D, IntersectionType)> {
    if !are_points_coplanar(p_u, p_v, q_u, q_v) {
        return None;
    }

    let p = p_v - p_u;
    let q = q_v - q_u;
    let normal_vector = p.cross(&q).normalize();
    let reference_point = p_u;
    let plane = (p.normalize(), p.cross(&normal_vector).normalize());

    calculate_2d_lineseg_intersection(
        project_point_onto_plane(p_u, plane, reference_point),
        project_point_onto_plane(p_v, plane, reference_point),
        project_point_onto_plane(q_u, plane, reference_point),
        project_point_onto_plane(q_v, plane, reference_point),
    )
    .map(|(point_in_2d, intersection_type)| {
        let point_in_3d = reference_point + (plane.0 * point_in_2d.x) + (plane.1 * point_in_2d.y);
        (point_in_3d, intersection_type)
    })
}

/// Calculates the distance of point `p` to triangle `t`
#[must_use]
pub fn distance_to_triangle(p: Vector3D, t: (Vector3D, Vector3D, Vector3D)) -> f64 {
    let (a, b, c) = t;

    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    // Normal vector of the triangle plane
    let n = ab.cross(&ac);
    let n_norm = n.norm();
    let n_unit = n / n_norm;

    // Distance from p to the plane
    let dist_to_plane = ap.dot(&n_unit);
    let proj = p - dist_to_plane * n_unit;

    if n_norm != 0.0 && is_point_inside_triangle(proj, (a, b, c)) {
        dist_to_plane.abs() // Perpendicular distance
    } else {
        // Closest distance to one of the triangle’s edges
        let d1 = distance_to_segment(p, a, b);
        let d2 = distance_to_segment(p, b, c);
        let d3 = distance_to_segment(p, c, a);
        d1.min(d2).min(d3)
    }
}

fn distance_to_segment(p: Vector3D, a: Vector3D, b: Vector3D) -> f64 {
    let ab = b - a;
    let t = (p - a).dot(&ab) / ab.dot(&ab);
    let t_clamped = t.clamp(0.0, 1.0);
    let closest = a + ab * t_clamped;
    (p - closest).norm()
}
