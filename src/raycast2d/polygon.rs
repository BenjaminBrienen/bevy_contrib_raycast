use crate::{RayCast2d, RayIntersection2d};

use bevy::math::{
    primitives::{BoxedPolygon, Direction2d, Polygon, RegularPolygon, Segment2d},
    Ray2d,
};

impl<const N: usize> RayCast2d for Polygon<N> {
    fn cast_ray_local(&self, ray: Ray2d, max_distance: f32) -> Option<RayIntersection2d> {
        let mut closest_intersection: Option<RayIntersection2d> = None;

        // Iterate through vertices to create edges
        for i in 0..self.vertices.len() {
            let start = self.vertices[i];
            let end = if i == self.vertices.len() - 1 {
                // Connect the last vertex to the first vertex to close the polygon
                self.vertices[0]
            } else {
                self.vertices[i + 1]
            };

            // Create the edge
            let segment =
                Segment2d::new(Direction2d::new(end - start).unwrap(), start.distance(end));

            // Cast the ray against the edge
            if let Some(intersection) =
                segment.cast_ray((start + end) / 2.0, 0.0, ray, max_distance)
            {
                if let Some(ref closest) = closest_intersection {
                    if intersection.distance < closest.distance {
                        closest_intersection = Some(intersection);
                    }
                } else {
                    closest_intersection = Some(intersection);
                }
            }
        }

        closest_intersection
    }
}

impl RayCast2d for BoxedPolygon {
    fn cast_ray_local(&self, ray: Ray2d, max_distance: f32) -> Option<RayIntersection2d> {
        let mut closest_intersection: Option<RayIntersection2d> = None;

        // Iterate through vertices to create edges
        for i in 0..self.vertices.len() {
            let start = self.vertices[i];
            let end = if i == self.vertices.len() - 1 {
                // Connect the last vertex to the first vertex to close the polygon
                self.vertices[0]
            } else {
                self.vertices[i + 1]
            };

            // Create the edge
            let segment =
                Segment2d::new(Direction2d::new(end - start).unwrap(), start.distance(end));

            // Cast the ray against the edge
            if let Some(intersection) =
                segment.cast_ray((start + end) / 2.0, 0.0, ray, max_distance)
            {
                if let Some(ref closest) = closest_intersection {
                    if intersection.distance < closest.distance {
                        closest_intersection = Some(intersection);
                    }
                } else {
                    closest_intersection = Some(intersection);
                }
            }
        }

        closest_intersection
    }
}

impl RayCast2d for RegularPolygon {
    fn cast_ray_local(&self, ray: Ray2d, max_distance: f32) -> Option<RayIntersection2d> {
        let poly = BoxedPolygon::from_iter(self.vertices(0.0));

        poly.cast_ray_local(ray, max_distance)
    }
}
