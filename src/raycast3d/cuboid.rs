use crate::{RayCast3d, RayIntersection3d};

use bevy::math::{primitives::Cuboid, Dir3, Ray3d};

impl RayCast3d for Cuboid {
    fn cast_ray_local(&self, ray: Ray3d, max_distance: f32) -> Option<RayIntersection3d> {
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

    fn create_cuboid() -> Cuboid {
        // Create a simple cuboid centered at the origin, with size 2x2x2
        Cuboid::new(2.0, 2.0, 2.0)
    }

    #[test]
    fn test_ray_misses_to_the_left() {
        let cuboid = create_cuboid();
        let ray = Ray3d {
            origin: Vec3::new(-3., 0., 5.), // Start in front of and to the left of the cuboid
            direction: Dir3::Z, // Moving forward
        };
        let result = cuboid.cast_ray_local(ray, 10.0);
        assert!(result.is_none(), "Ray should miss the cuboid to the left");
    }

    #[test]
    fn test_ray_misses_to_the_right() {
        let cuboid = create_cuboid();
        let ray = Ray3d {
            origin: Vec3::new(3., 0., 5.), // Start in front of and to the right of the cuboid
            direction: Dir3::Z, // Moving forward
        };
        let result = cuboid.cast_ray_local(ray, 10.0);
        assert!(result.is_none(), "Ray should miss the cuboid to the right");
    }

    #[test]
    fn test_ray_misses_above() {
        let cuboid = create_cuboid();
        let ray = Ray3d {
            origin: Vec3::new(0., 3., 5.), // Start in front of and above the cuboid
            direction: Dir3::Z, // Moving forward
        };
        let result = cuboid.cast_ray_local(ray, 10.0);
        assert!(result.is_none(), "Ray should miss the cuboid above");
    }

    #[test]
    fn test_ray_misses_below() {
        let cuboid = create_cuboid();
        let ray = Ray3d {
            origin: Vec3::new(0., -3., 5.), // Start in front of and below the cuboid
            direction: Dir3::NEG_Y, // Moving downwards
        };
        let result = cuboid.cast_ray_local(ray, 10.0);
        assert!(result.is_none(), "Ray should miss the cuboid below");
    }

    #[test]
    fn test_ray_misses_too_short_distance() {
        let cuboid = create_cuboid();
        let ray = Ray3d {
            origin: Vec3::new(0., 0., 5.), // Start directly in front of the cuboid
            direction: Dir3::NEG_Z, // Moving towards the cuboid
        };
        let result = cuboid.cast_ray_local(ray, 1.0); // Max distance is too short to reach the cuboid
        assert!(result.is_none(), "Ray should miss the cuboid due to short max distance");
    }

    #[test]
    fn test_ray_hits_cuboid() {
        let cuboid = create_cuboid();
        let ray = Ray3d {
            origin: Vec3::new(0., 0., 5.), // Start directly in front of the cuboid
            direction: Dir3::NEG_Z, // Moving towards the cuboid
        };
        let result = cuboid.cast_ray_local(ray, 10.0);
        assert!(result.is_some(), "Ray should hit the cuboid");
        let intersection = result.unwrap();
        assert!(intersection.distance > 0.0, "Intersection distance should be positive");
        assert_eq!(intersection.normal, Dir3::Z, "Normal should point along the Z-axis (back where the ray came from)");
    }
}
