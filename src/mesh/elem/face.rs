use crate::prelude::*;
use core::panic;
use itertools::Itertools;

impl<M: Tag> Mesh<M> {
    #[must_use]
    pub fn frep(&self, id: FaceKey<M>) -> EdgeKey<M> {
        self.face_repr.get(id).unwrap_or_else(|| panic!("{id:?} has no frep"))
    }

    // Returns the two edges of a given face that are connected to the given vertex.
    #[must_use]
    pub fn edges_in_face_with_vert(&self, face_id: FaceKey<M>, vert_id: VertKey<M>) -> Option<[EdgeKey<M>; 2]> {
        let edges = self.edges(face_id);
        edges
            .into_iter()
            .filter(|&edge_id| self.root(edge_id) == vert_id || self.toor(edge_id) == vert_id)
            .collect_tuple()
            .map(|(a, b)| [a, b])
    }

    // Returns the edge between the two faces. Returns None if the faces do not share an edge.
    #[must_use]
    pub fn edge_between_faces(&self, id_a: FaceKey<M>, id_b: FaceKey<M>) -> Option<(EdgeKey<M>, EdgeKey<M>)> {
        let edges_a = self.edges(id_a);
        let edges_b = self.edges(id_b);
        for &edge_a_id in &edges_a {
            for &edge_b_id in &edges_b {
                if self.twin(edge_a_id) == edge_b_id {
                    return Some((edge_a_id, edge_b_id));
                }
            }
        }
        None
    }

    // Returns the face with given vertices.
    #[must_use]
    pub fn face_with_verts(&self, verts: &[VertKey<M>]) -> Option<FaceKey<M>> {
        self.faces(verts[0])
            .into_iter()
            .find(|&face_id| verts.iter().all(|&vert_id| self.faces(vert_id).contains(&face_id)))
    }

    // Vector area of a given face.
    #[must_use]
    pub fn vector_area(&self, id: FaceKey<M>) -> Vector3D {
        self.edges(id).iter().fold(Vector3D::zeros(), |sum, &edge_id| {
            let u = self.vector(self.twin(edge_id));
            let v = self.vector(self.next(edge_id));
            sum + u.cross(&v)
        })
    }
}

impl<M: Tag> HasPosition<FACE, M> for Mesh<M> {
    // Get centroid of a given polygonal face.
    // https://en.wikipedia.org/wiki/Centroid
    // Be careful with concave faces, the centroid might lay outside the face.
    fn position(&self, id: FaceKey<M>) -> Vector3D {
        math::calculate_average_f64(self.edges(id).iter().map(|&edge_id| self.position(self.root(edge_id))))
    }
}

impl<M: Tag> HasNormal<FACE, M> for Mesh<M> {
    fn normal(&self, id: FaceKey<M>) -> Vector3D {
        // TODO: Make this better for non-planar faces.
        let vertices = self.vertices(id);
        let (u, v, w) = (vertices[0], vertices[1], vertices[2]);
        let p_u = self.position(u);
        let p_v = self.position(v);
        let p_w = self.position(w);
        let p = p_v - p_u;
        let q = p_w - p_u;
        let normal_vector = p.cross(&q).normalize();
        normal_vector
    }
}

impl<M: Tag> HasSize<FACE, M> for Mesh<M> {
    // Area of a given face.
    fn size(&self, id: FaceKey<M>) -> Float {
        self.vector_area(id).magnitude() / 2.0
    }
}

impl<M: Tag> HasVertices<FACE, M> for Mesh<M> {
    fn vertices(&self, id: FaceKey<M>) -> Vec<VertKey<M>> {
        self.edges(id).into_iter().map(|edge_id| self.root(edge_id)).collect()
    }
}

impl<M: Tag> HasEdges<FACE, M> for Mesh<M> {
    fn edges(&self, id: FaceKey<M>) -> Vec<EdgeKey<M>> {
        [vec![self.frep(id)], self.neighbors(self.frep(id))].concat()
    }
}

impl<M: Tag> HasNeighbors<FACE, M> for Mesh<M> {
    fn neighbors(&self, id: FaceKey<M>) -> Vec<FaceKey<M>> {
        self.edges(id).into_iter().map(|edge_id| self.face(self.twin(edge_id))).collect()
    }
}
