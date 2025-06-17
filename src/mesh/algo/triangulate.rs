use std::collections::HashMap;

use crate::prelude::*;
use earcutr;

// Given an arbitrary polygonal mesh, triangulate all faces to obtain a triangular mesh.
// This is useful for rendering or further processing.
impl<M: Tag> Mesh<M> {
    pub fn triangulate(&mut self) -> Result<Self, MeshError<M>> {
        let mut new_mesh = self.clone();

        for face in self.face_ids() {
            let degree = self.vertices(face).len();
            match degree {
                0..=2 => return Err(MeshError::FaceNotPolygon(face)),
                3 => continue, // Already a triangle
                _ => {
                    self.triangulate_face(face)?;
                }
            }
        }

        Ok(new_mesh)
    }

    pub fn triangulate_face(&mut self, face: FaceKey<M>) -> Result<(), MeshError<M>> {
        let vertices = self.vertices(face);
        let positions = vertices.iter().map(|&v| self.position(v)).map(|v| vec![v.x, v.y, v.z]).collect::<Vec<_>>();

        let v = vec![
            positions, // outer ring
            vec![],    // hole ring
        ];
        let (flattened_positions, holes, dimensions) = earcutr::flatten(&v);
        let binding = earcutr::earcut(&flattened_positions, &holes, dimensions).unwrap();
        let triangles = binding.windows(3).collect::<Vec<_>>();

        // Remove the old face
        self.faces.remove(face);

        let mut endpoints_to_edges = HashMap::<(VertKey<M>, VertKey<M>), EdgeKey<M>>::new();
        for triangle in triangles {
            let face_id = self.add_face();

            let mut edge_ids = vec![];

            // Find correct orientation of the triangle
            let triangle_e1 = (*vertices.get(triangle[0]).unwrap(), *vertices.get(triangle[1]).unwrap());
            let triangle_e2 = (*vertices.get(triangle[1]).unwrap(), *vertices.get(triangle[2]).unwrap());
            let triangle_e3 = (*vertices.get(triangle[2]).unwrap(), *vertices.get(triangle[0]).unwrap());

            let mut reversed = false;

            // Atleast one of these edges must already exist in the mesh, with `face` as one its face.
            if let Some((e1, e2)) = self.edge_between_verts(triangle_e1.0, triangle_e1.1) {
                if self.face(e2) == face {
                    reversed = true;
                }
            } else if let Some((e1, e2)) = self.edge_between_verts(triangle_e2.0, triangle_e2.1) {
                if self.face(e2) == face {
                    reversed = true;
                }
            } else if let Some((e1, e2)) = self.edge_between_verts(triangle_e3.0, triangle_e3.1) {
                if self.face(e2) == face {
                    reversed = true;
                }
            }

            // If reversed, we need to reverse the triangle
            let triangle = if reversed {
                vec![triangle[0], triangle[2], triangle[1]]
            } else {
                triangle.to_vec()
            };

            for i in 0..3 {
                let v1 = triangle[i];
                let v2 = triangle[(i + 1) % triangle.len()];
                let (&start_vertex, &end_vertex) = (vertices.get(v1).unwrap(), vertices.get(v2).unwrap());
                // Check if edge already exists
                if let Some((e1, e2)) = self.edge_between_verts(start_vertex, end_vertex) {
                    let f1 = self.face(e1);
                    if f1 == face {
                        edge_ids.push(e1);
                    } else {
                        // Never true
                        unreachable!();
                    }
                } else {
                    edge_ids.push(self.add_edge());
                }

                let edge_id = edge_ids.last().unwrap().to_owned();
                if endpoints_to_edges.insert((start_vertex, end_vertex), edge_id).is_some() {
                    return Err(MeshError::DuplicateEdge(start_vertex, end_vertex));
                }
                self.face_repr.insert(face_id, edge_id);
                self.vert_repr.insert(start_vertex, edge_id);
                self.edge_root.insert(edge_id, start_vertex);
                self.edge_face.insert(edge_id, face_id);
            }

            // Linking each edge to its next edge in the face
            for edge_index in 0..edge_ids.len() {
                self.edge_next.insert(edge_ids[edge_index], edge_ids[(edge_index + 1) % edge_ids.len()]);
            }
        }

        for edge_id in self.edge_ids() {
            let endpoints = self.vertices(edge_id);
            let (e1, e2) = self.edge_between_verts(endpoints[0], endpoints[1]).unwrap();
            assert!(e1 == edge_id);
            self.edge_twin.insert(edge_id, e2);
            self.edge_twin.insert(e2, edge_id);
        }

        Ok(())
    }
}
