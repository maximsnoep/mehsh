use crate::prelude::*;
use bvh::{
    aabb::{Aabb, Bounded},
    bounding_hierarchy::BHShape,
    bvh::Bvh,
    point_query::PointDistance,
};
use itertools::Itertools;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaceLocation<M: Tag>((Bvh<f64, 3>, Vec<TriangleBvhShape<M>>));
impl<M: Tag> FaceLocation<M> {
    #[must_use]
    pub fn nearest(&self, point: &[f64; 3]) -> FaceKey<M> {
        let neighbor = self.0.0.nearest_to(nalgebra::Point3::from_slice(point), &self.0.1);
        let (t, _) = neighbor.unwrap();
        t.real_index
    }
    fn overwrite(&mut self, shapes: &mut [TriangleBvhShape<M>]) {
        self.0.0 = Bvh::build(shapes);
        self.0.1 = shapes.to_vec();
    }
}
impl<M: Tag> Default for FaceLocation<M> {
    fn default() -> Self {
        Self((Bvh::build::<TriangleBvhShape<M>>(&mut []), Vec::new()))
    }
}

impl<M: Tag> Mesh<M> {
    #[must_use]
    pub fn bvh(&self) -> FaceLocation<M> {
        let mut bvh = FaceLocation::default();
        let mut triangles = self
            .face_ids()
            .iter()
            .enumerate()
            .map(|(i, &face_id)| TriangleBvhShape {
                corners: [
                    self.position(self.vertices(face_id)[0]),
                    self.position(self.vertices(face_id)[1]),
                    self.position(self.vertices(face_id)[2]),
                ],
                node_index: i,
                real_index: face_id,
            })
            .collect_vec();

        bvh.overwrite(&mut triangles);
        bvh
    }
}

// impl for triangles

#[derive(Debug, Clone, Serialize, Deserialize)]
struct TriangleBvhShape<M: Tag> {
    corners: [Vector3D; 3],
    node_index: usize,
    real_index: FaceKey<M>,
}

impl<M: Tag> PointDistance<f64, 3> for TriangleBvhShape<M> {
    fn distance_squared(&self, query_point: nalgebra::Point<f64, 3>) -> f64 {
        geom::distance_to_triangle(
            Vector3D::new(query_point[0], query_point[1], query_point[2]),
            (self.corners[0], self.corners[1], self.corners[2]),
        )
    }
}

impl<M: Tag> Bounded<f64, 3> for TriangleBvhShape<M> {
    fn aabb(&self) -> Aabb<f64, 3> {
        let min_x = self.corners.iter().map(|v| v.x).fold(f64::MAX, f64::min);
        let min_y = self.corners.iter().map(|v| v.y).fold(f64::MAX, f64::min);
        let min_z = self.corners.iter().map(|v| v.z).fold(f64::MAX, f64::min);
        let max_x = self.corners.iter().map(|v| v.x).fold(f64::MIN, f64::max);
        let max_y = self.corners.iter().map(|v| v.y).fold(f64::MIN, f64::max);
        let max_z = self.corners.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        let min = nalgebra::Point3::new(min_x, min_y, min_z);
        let max = nalgebra::Point3::new(max_x, max_y, max_z);
        Aabb::with_bounds(min, max)
    }
}

impl<M: Tag> BHShape<f64, 3> for TriangleBvhShape<M> {
    fn set_bh_node_index(&mut self, index: usize) {
        self.node_index = index;
    }

    fn bh_node_index(&self) -> usize {
        self.node_index
    }
}
