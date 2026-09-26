use crate::linear_math::{Aabb, Vector3};
use std::collections::{HashMap, HashSet};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// An overlapping pair of body handles detected by the broadphase.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BroadphasePair {
    pub body_a: usize,
    pub body_b: usize,
}

impl BroadphasePair {
    #[inline]
    pub fn new(a: usize, b: usize) -> Self {
        if a < b {
            Self { body_a: a, body_b: b }
        } else {
            Self { body_a: b, body_b: a }
        }
    }
}

/// Node in the dynamic bounding volume tree.
#[derive(Debug, Clone)]
struct DbvtNode {
    aabb: Aabb,
    parent: Option<usize>,
    child1: Option<usize>,
    child2: Option<usize>,
    body_id: Option<usize>,
}

impl DbvtNode {
    #[inline]
    fn is_leaf(&self) -> bool {
        self.child1.is_none()
    }
}

/// Dynamic Bounding Volume Tree (DBVT) broadphase collision detector.
/// Direct equivalent to Bullet's `btDbvtBroadphase`.
#[derive(Debug, Clone)]
pub struct DbvtBroadphase {
    nodes: Vec<Option<DbvtNode>>,
    free_nodes: Vec<usize>,
    root: Option<usize>,
    body_to_leaf: HashMap<usize, usize>,
    pair_cache: HashSet<BroadphasePair>,
}

impl DbvtBroadphase {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            free_nodes: Vec::new(),
            root: None,
            body_to_leaf: HashMap::new(),
            pair_cache: HashSet::new(),
        }
    }

    pub fn insert(&mut self, body_id: usize, aabb: Aabb) {
        if let Some(&_existing_leaf) = self.body_to_leaf.get(&body_id) {
            self.update(body_id, aabb);
            return;
        }

        // Expand AABB slightly with velocity/predictive margin to minimize rebuilds
        let expanded = aabb.expanded(0.05);
        let leaf_idx = self.allocate_node(DbvtNode {
            aabb: expanded,
            parent: None,
            child1: None,
            child2: None,
            body_id: Some(body_id),
        });

        self.body_to_leaf.insert(body_id, leaf_idx);
        self.insert_leaf(leaf_idx);
    }

    pub fn update(&mut self, body_id: usize, aabb: Aabb) {
        if let Some(&leaf_idx) = self.body_to_leaf.get(&body_id) {
            if let Some(Some(node)) = self.nodes.get(leaf_idx) {
                // If new AABB is still within the fat AABB, avoid tree rebalance
                if node.aabb.contains_aabb(&aabb) {
                    return;
                }
            }
            self.remove_leaf(leaf_idx);
            let expanded = aabb.expanded(0.05);
            if let Some(Some(node)) = self.nodes.get_mut(leaf_idx) {
                node.aabb = expanded;
            }
            self.insert_leaf(leaf_idx);
        } else {
            self.insert(body_id, aabb);
        }
    }

    pub fn remove(&mut self, body_id: usize) {
        if let Some(leaf_idx) = self.body_to_leaf.remove(&body_id) {
            self.remove_leaf(leaf_idx);
            self.free_node(leaf_idx);
        }
    }

    fn allocate_node(&mut self, node: DbvtNode) -> usize {
        if let Some(idx) = self.free_nodes.pop() {
            self.nodes[idx] = Some(node);
            idx
        } else {
            let idx = self.nodes.len();
            self.nodes.push(Some(node));
            idx
        }
    }

    fn free_node(&mut self, idx: usize) {
        if idx < self.nodes.len() {
            self.nodes[idx] = None;
            self.free_nodes.push(idx);
        }
    }

    fn insert_leaf(&mut self, leaf: usize) {
        if self.root.is_none() {
            self.root = Some(leaf);
            return;
        }

        let leaf_aabb = self.nodes[leaf].as_ref().unwrap().aabb;
        let mut current = self.root.unwrap();

        // Traverse down the tree to find the best sibling
        while !self.nodes[current].as_ref().unwrap().is_leaf() {
            let curr_node = self.nodes[current].as_ref().unwrap();
            let c1 = curr_node.child1.unwrap();
            let c2 = curr_node.child2.unwrap();

            let aabb1 = self.nodes[c1].as_ref().unwrap().aabb;
            let aabb2 = self.nodes[c2].as_ref().unwrap().aabb;

            let merged1 = aabb1.merge(&leaf_aabb);
            let merged2 = aabb2.merge(&leaf_aabb);

            let cost1 = merged1.extents().length_squared();
            let cost2 = merged2.extents().length_squared();

            if cost1 < cost2 {
                current = c1;
            } else {
                current = c2;
            }
        }

        let sibling = current;
        let old_parent = self.nodes[sibling].as_ref().unwrap().parent;

        // Create new internal parent
        let sibling_aabb = self.nodes[sibling].as_ref().unwrap().aabb;
        let new_parent = self.allocate_node(DbvtNode {
            aabb: sibling_aabb.merge(&leaf_aabb),
            parent: old_parent,
            child1: Some(sibling),
            child2: Some(leaf),
            body_id: None,
        });

        self.nodes[sibling].as_mut().unwrap().parent = Some(new_parent);
        self.nodes[leaf].as_mut().unwrap().parent = Some(new_parent);

        if let Some(op) = old_parent {
            let op_node = self.nodes[op].as_mut().unwrap();
            if op_node.child1 == Some(sibling) {
                op_node.child1 = Some(new_parent);
            } else {
                op_node.child2 = Some(new_parent);
            }
        } else {
            self.root = Some(new_parent);
        }

        // Walk back up refitting ancestors
        let mut walk = Some(new_parent);
        while let Some(w) = walk {
            let c1 = self.nodes[w].as_ref().unwrap().child1.unwrap();
            let c2 = self.nodes[w].as_ref().unwrap().child2.unwrap();
            let a1 = self.nodes[c1].as_ref().unwrap().aabb;
            let a2 = self.nodes[c2].as_ref().unwrap().aabb;
            self.nodes[w].as_mut().unwrap().aabb = a1.merge(&a2);
            walk = self.nodes[w].as_ref().unwrap().parent;
        }
    }

    fn remove_leaf(&mut self, leaf: usize) {
        if self.root == Some(leaf) {
            self.root = None;
            return;
        }

        let parent = self.nodes[leaf].as_ref().unwrap().parent.unwrap();
        let parent_node = self.nodes[parent].as_ref().unwrap();
        let grand_parent = parent_node.parent;
        let sibling = if parent_node.child1 == Some(leaf) {
            parent_node.child2.unwrap()
        } else {
            parent_node.child1.unwrap()
        };

        if let Some(gp) = grand_parent {
            let gp_node = self.nodes[gp].as_mut().unwrap();
            if gp_node.child1 == Some(parent) {
                gp_node.child1 = Some(sibling);
            } else {
                gp_node.child2 = Some(sibling);
            }
            self.nodes[sibling].as_mut().unwrap().parent = Some(gp);
            self.free_node(parent);

            let mut walk = Some(gp);
            while let Some(w) = walk {
                let c1 = self.nodes[w].as_ref().unwrap().child1;
                let c2 = self.nodes[w].as_ref().unwrap().child2;
                if let (Some(c1_idx), Some(c2_idx)) = (c1, c2) {
                    let a1 = self.nodes[c1_idx].as_ref().unwrap().aabb;
                    let a2 = self.nodes[c2_idx].as_ref().unwrap().aabb;
                    self.nodes[w].as_mut().unwrap().aabb = a1.merge(&a2);
                }
                walk = self.nodes[w].as_ref().unwrap().parent;
            }
        } else {
            self.root = Some(sibling);
            self.nodes[sibling].as_mut().unwrap().parent = None;
            self.free_node(parent);
        }
    }

    /// Compute all candidate overlapping pairs.
    pub fn compute_pairs(&mut self) -> Vec<BroadphasePair> {
        let mut pairs = Vec::new();
        if let Some(root_idx) = self.root {
            self.query_tree_internal(root_idx, &mut pairs);
        }
        self.pair_cache.clear();
        for &p in &pairs {
            self.pair_cache.insert(p);
        }
        pairs
    }

    fn query_tree_internal(&self, node_idx: usize, pairs: &mut Vec<BroadphasePair>) {
        let node = match &self.nodes[node_idx] {
            Some(n) => n,
            None => return,
        };

        if node.is_leaf() {
            return;
        }

        let c1 = node.child1.unwrap();
        let c2 = node.child2.unwrap();

        self.query_tree_internal(c1, pairs);
        self.query_tree_internal(c2, pairs);
        self.query_two_nodes(c1, c2, pairs);
    }

    fn query_two_nodes(&self, a_idx: usize, b_idx: usize, pairs: &mut Vec<BroadphasePair>) {
        let a = match &self.nodes[a_idx] {
            Some(n) => n,
            None => return,
        };
        let b = match &self.nodes[b_idx] {
            Some(n) => n,
            None => return,
        };

        if !a.aabb.intersects(&b.aabb) {
            return;
        }

        if a.is_leaf() && b.is_leaf() {
            if let (Some(ba), Some(bb)) = (a.body_id, b.body_id) {
                if ba != bb {
                    pairs.push(BroadphasePair::new(ba, bb));
                }
            }
        } else if a.is_leaf() {
            self.query_two_nodes(a_idx, b.child1.unwrap(), pairs);
            self.query_two_nodes(a_idx, b.child2.unwrap(), pairs);
        } else if b.is_leaf() {
            self.query_two_nodes(a.child1.unwrap(), b_idx, pairs);
            self.query_two_nodes(a.child2.unwrap(), b_idx, pairs);
        } else {
            self.query_two_nodes(a.child1.unwrap(), b.child1.unwrap(), pairs);
            self.query_two_nodes(a.child1.unwrap(), b.child2.unwrap(), pairs);
            self.query_two_nodes(a.child2.unwrap(), b.child1.unwrap(), pairs);
            self.query_two_nodes(a.child2.unwrap(), b.child2.unwrap(), pairs);
        }
    }

    /// Ray test query against all broadphase AABBs.
    pub fn ray_test(&self, ray_from: Vector3, ray_to: Vector3) -> Vec<usize> {
        let mut results = Vec::new();
        if let Some(root_idx) = self.root {
            self.ray_test_recursive(root_idx, ray_from, ray_to, &mut results);
        }
        results
    }

    fn ray_test_recursive(&self, node_idx: usize, ray_from: Vector3, ray_to: Vector3, results: &mut Vec<usize>) {
        let node = match &self.nodes[node_idx] {
            Some(n) => n,
            None => return,
        };

        if node.aabb.ray_test(ray_from, ray_to).is_none() {
            return;
        }

        if node.is_leaf() {
            if let Some(body_id) = node.body_id {
                results.push(body_id);
            }
        } else {
            if let Some(c1) = node.child1 {
                self.ray_test_recursive(c1, ray_from, ray_to, results);
            }
            if let Some(c2) = node.child2 {
                self.ray_test_recursive(c2, ray_from, ray_to, results);
            }
        }
    }
}

impl Default for DbvtBroadphase {
    fn default() -> Self {
        Self::new()
    }
}
