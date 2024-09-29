use crate::{RayCast3d, RayIntersection3d};

use bevy::math::{primitives::Cuboid, Dir3, Ray3d};

impl RayCast3d for Cuboid {
    fn cast_ray_local(&self, ray: Ray3d, max_distance: f32) -> Option<RayIntersection3d> {
        // TODO: make this work for non-axis-aligned cuboids (transformation matrix?)
        let inv_dir = 1.0 / ray.direction.as_vec3();
        let min_bounds = -self.half_size;
        let max_bounds = self.half_size;

        let t1 = (min_bounds.x - ray.origin.x) * inv_dir.x;
        let t2 = (max_bounds.x - ray.origin.x) * inv_dir.x;
        let t3 = (min_bounds.y - ray.origin.y) * inv_dir.y;
        let t4 = (max_bounds.y - ray.origin.y) * inv_dir.y;
        let t5 = (min_bounds.z - ray.origin.z) * inv_dir.z;
        let t6 = (max_bounds.z - ray.origin.z) * inv_dir.z;

        let tmin = t1.min(t2).max(t3.min(t4)).max(t5.min(t6));
        let tmax = t1.max(t2).min(t3.max(t4)).min(t5.max(t6));

        if tmax < 0.0 || tmin > tmax {
            // No intersection
            return None;
        }

        let t = if tmin < 0.0 { tmax } else { tmin };
        if t > max_distance {
            return None;
        }

        let intersection = ray.origin + t * ray.direction;

        // Determine the normal at the intersection point based on which axis was hit
        // TODO: this is the suspicious code where the tests fail
        let normal = if t == t1 || t == t2 {
            t1.signum() * Dir3::X
        } else if t == t3 || t == t4 {
            t3.signum() * Dir3::Y
        } else {
            t5.signum() * Dir3::Z
        };

        Some(RayIntersection3d {
            normal: Dir3::new_unchecked(normal),
            position: intersection,
            distance: t,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::Vec3;
    use test_case::test_case;

    enum Test
    {
        Hit,
        Misfire(Dir3),
    }

    fn do_test(aim_axis: Dir3, test: Test) -> Option<RayIntersection3d> {
        // Values chosen so that when the misfire and standoff are on the same axis the ray will miss from either side
        const CUBOID_X_SIZE: f32 = 2.0; // TODO: test different proportioned cuboids
        const CUBOID_Y_SIZE: f32 = 2.0; // TODO: test different proportioned cuboids
        const CUBOID_Z_SIZE: f32 = 2.0; // TODO: test different proportioned cuboids
        const MISFIRE_MAGNITUDE: f32 = 10. + CUBOID_X_SIZE;
        const STANDOFF_MAGNITUDE: f32 = 10.;
        const MAX_DISTANCE: f32 = STANDOFF_MAGNITUDE;
        let cuboid = Cuboid::new(CUBOID_X_SIZE, CUBOID_Y_SIZE, CUBOID_Z_SIZE);
        let misfire_offset = if let Test::Misfire(axis) = test
        {
            MISFIRE_MAGNITUDE * axis
        } else{
            Vec3::ZERO
        };
        // stand back from the origin (looking towards the origin)
        let stand_off = -STANDOFF_MAGNITUDE * aim_axis;
        let ray = Ray3d {
            origin: stand_off + misfire_offset,
            direction: aim_axis,
        };

        cuboid.cast_ray_local(ray, MAX_DISTANCE)
    }

    #[test_case(Dir3::X, Dir3::X ; "when X ray is too far X")]
    #[test_case(Dir3::X, Dir3::Y ; "when X ray is too far Y")]
    #[test_case(Dir3::X, Dir3::Z ; "when X ray is too far Z")]
    #[test_case(Dir3::X, Dir3::NEG_X ; "when X ray is too far NegX")]
    #[test_case(Dir3::X, Dir3::NEG_Y ; "when X ray is too far NegY")]
    #[test_case(Dir3::X, Dir3::NEG_Z ; "when X ray is too far NegZ")]

    #[test_case(Dir3::Y, Dir3::X ; "when Y ray is too far X")]
    #[test_case(Dir3::Y, Dir3::Y ; "when Y ray is too far Y")]
    #[test_case(Dir3::Y, Dir3::Z ; "when Y ray is too far Z")]
    #[test_case(Dir3::Y, Dir3::NEG_X ; "when Y ray is too far NegX")]
    #[test_case(Dir3::Y, Dir3::NEG_Y ; "when Y ray is too far NegY")]
    #[test_case(Dir3::Y, Dir3::NEG_Z ; "when Y ray is too far NegZ")]

    #[test_case(Dir3::Z, Dir3::X ; "when Z ray is too far X")]
    #[test_case(Dir3::Z, Dir3::Y ; "when Z ray is too far Y")]
    #[test_case(Dir3::Z, Dir3::Z ; "when Z ray is too far Z")]
    #[test_case(Dir3::Z, Dir3::NEG_X ; "when Z ray is too far NegX")]
    #[test_case(Dir3::Z, Dir3::NEG_Y ; "when Z ray is too far NegY")]
    #[test_case(Dir3::Z, Dir3::NEG_Z ; "when Z ray is too far NegZ")]

    #[test_case(Dir3::NEG_X, Dir3::X ; "when NegX ray is too far X")]
    #[test_case(Dir3::NEG_X, Dir3::Y ; "when NegX ray is too far Y")]
    #[test_case(Dir3::NEG_X, Dir3::Z ; "when NegX ray is too far Z")]
    #[test_case(Dir3::NEG_X, Dir3::NEG_X ; "when NegX ray is too far NegX")]
    #[test_case(Dir3::NEG_X, Dir3::NEG_Y ; "when NegX ray is too far NegY")]
    #[test_case(Dir3::NEG_X, Dir3::NEG_Z ; "when NegX ray is too far NegZ")]

    #[test_case(Dir3::NEG_Y, Dir3::X ; "when NegY ray is too far X")]
    #[test_case(Dir3::NEG_Y, Dir3::Y ; "when NegY ray is too far Y")]
    #[test_case(Dir3::NEG_Y, Dir3::Z ; "when NegY ray is too far Z")]
    #[test_case(Dir3::NEG_Y, Dir3::NEG_X ; "when NegY ray is too far NegX")]
    #[test_case(Dir3::NEG_Y, Dir3::NEG_Y ; "when NegY ray is too far NegY")]
    #[test_case(Dir3::NEG_Y, Dir3::NEG_Z ; "when NegY ray is too far NegZ")]

    #[test_case(Dir3::NEG_Z, Dir3::X ; "when NegZ ray is too far X")]
    #[test_case(Dir3::NEG_Z, Dir3::Y ; "when NegZ ray is too far Y")]
    #[test_case(Dir3::NEG_Z, Dir3::Z ; "when NegZ ray is too far Z")]
    #[test_case(Dir3::NEG_Z, Dir3::NEG_X ; "when NegZ ray is too far NegX")]
    #[test_case(Dir3::NEG_Z, Dir3::NEG_Y ; "when NegZ ray is too far NegY")]
    #[test_case(Dir3::NEG_Z, Dir3::NEG_Z ; "when NegZ ray is too far NegZ")]
    fn misses(aim_axis: Dir3, miss_side: Dir3) {
        let result = do_test(aim_axis, Test::Misfire(miss_side));
        assert!(result.is_none(), "Ray should miss the cuboid");
    }

    #[test_case(Dir3::X ; "when X ray is just right")]
    #[test_case(Dir3::Y ; "when Y ray is just right")]
    #[test_case(Dir3::Z ; "when Z ray is just right")]
    #[test_case(Dir3::NEG_X ; "when NegX ray is just right")] // TODO: fix
    #[test_case(Dir3::NEG_Y ; "when NegY ray is just right")] // TODO: fix
    #[test_case(Dir3::NEG_Z ; "when NegZ ray is just right")] // TODO: fix
    fn hits(aim_axis: Dir3) {
        let result = do_test(aim_axis, Test::Hit);
        assert!(result.is_some(), "Ray should hit the cuboid");
        let intersection = result.unwrap();
        assert!(intersection.distance > 0.0, "Intersection distance should be positive");
        assert_eq!(intersection.normal, aim_axis, "Normal should point back where the ray came from");
    }
}
