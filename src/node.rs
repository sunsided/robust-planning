//! Tree nodes that carry a reachable set, shared by [`RobustRrt`](crate::RobustRrt) and
//! [`ReachSearch`](crate::ReachSearch).

use polyzonotope::SparsePolyZonotope;
use sampling_planning::{Euclidean, Path, StateSpace};

/// A node of a planner tree: a nominal state, the reachable set there and the edge from its
/// parent.
#[derive(Debug, Clone)]
pub struct Node<C, const N: usize> {
    /// The nominal state the node's control drives toward.
    pub nominal: [f64; N],
    /// Every state the system can be in at this node.
    pub set: SparsePolyZonotope,
    /// The parent node id; `None` only for the root.
    pub parent: Option<usize>,
    /// The control from the parent to this node; `None` only for the root.
    pub control: Option<C>,
}

/// The node ids from the root to `node`, inclusive.
///
/// # Panics
///
/// Panics if `node` or one of its ancestors is not an index into `nodes`.
pub(crate) fn branch<C, const N: usize>(nodes: &[Node<C, N>], node: usize) -> Vec<usize> {
    let mut ids = vec![node];
    let mut cursor = node;

    while let Some(parent) = nodes[cursor].parent {
        ids.push(parent);
        cursor = parent;
    }

    ids.reverse();
    ids
}

/// The nominal path from the root to `node`, with its Euclidean length as the cost.
///
/// # Panics
///
/// Panics if `node` or one of its ancestors is not an index into `nodes`.
pub(crate) fn nominal_path<C, const N: usize>(nodes: &[Node<C, N>], node: usize) -> Path<[f64; N]> {
    let states: Vec<[f64; N]> = branch(nodes, node)
        .into_iter()
        .map(|id| nodes[id].nominal)
        .collect();
    let cost = states
        .windows(2)
        .map(|pair| Euclidean::<N>.distance(&pair[0], &pair[1]))
        .sum();

    Path { states, cost }
}
