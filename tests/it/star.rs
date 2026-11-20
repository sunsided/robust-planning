//! Robust RRT* on the slalom: audited edges, Monte-Carlo runs and cost over iterations.

use polyzonotope::Intersection;
use rand_pcg::Pcg64Mcg;
use robust_planning::Reach;
use sampling_planning::rand_core::SeedableRng;

use crate::{
    ITERATIONS, simulate,
    world::{SLALOM, SPLITS, StarPlanner, planner, polytopes, star_planner},
};

/// Iterations of every Robust RRT* run.
const STAR_ITERATIONS: usize = 1500;

/// A Robust RRT* on the slalom after [`STAR_ITERATIONS`] iterations with `seed`, and the
/// best cost after every iteration from the first solution on.
fn grown_slalom(seed: u64) -> (StarPlanner, Vec<f64>) {
    let mut planner = star_planner(&SLALOM, [1.0, 1.0], [9.0, 9.0]);
    let mut rng = Pcg64Mcg::seed_from_u64(seed);
    let mut costs = Vec::new();

    for _ in 0..STAR_ITERATIONS {
        planner.step(&mut rng);

        if let Some(path) = planner.path() {
            costs.push(path.cost);
        }
    }

    assert!(!costs.is_empty(), "seed {seed}: the slalom is solvable");
    #[expect(
        clippy::float_cmp,
        reason = "a zero-length edge repeats the state exactly"
    )]
    let repeats = planner
        .nodes()
        .iter()
        .skip(1)
        .any(|node| node.nominal == planner.nodes()[node.parent.expect("non-root")].nominal);
    assert!(!repeats, "seed {seed}: a zero-length edge was added");
    assert!(
        planner.stats().rewires > 0,
        "seed {seed}: no rewire committed"
    );

    (planner, costs)
}

#[test]
fn every_final_edge_is_proven_disjoint_from_its_parents_current_set() {
    let obstacles = polytopes(&SLALOM);

    for seed in 0..2 {
        let (planner, _) = grown_slalom(seed);
        let reach = &planner.problem().reach;
        let mut cover = Vec::new();

        for (id, node) in planner.nodes().iter().enumerate().skip(1) {
            let parent = &planner.nodes()[node.parent.expect("non-root")];
            let control = node.control.as_ref().expect("non-root");

            cover.clear();
            let end = reach.propagate(&parent.set, control, &mut cover);
            assert_ne!(cover.len(), 0, "every edge has a cover");
            assert_eq!(
                end.interval_hull(),
                node.set.interval_hull(),
                "seed {seed}: node {id} keeps a stale set"
            );

            for (set, obstacle) in cover
                .iter()
                .flat_map(|s| obstacles.iter().map(move |o| (s, o)))
            {
                assert_eq!(set.intersection(obstacle, SPLITS), Intersection::Disjoint);
            }
        }
    }
}

#[test]
fn monte_carlo_runs_along_the_returned_path_never_hit_obstacles() {
    let obstacles = polytopes(&SLALOM);
    let mut rng = Pcg64Mcg::seed_from_u64(42);

    for seed in 0..2 {
        let (planner, _) = grown_slalom(seed);
        let goal = planner.goal_node().expect("solved");
        let count = planner.nodes().len();
        let targets = (0..10).map(|k| k * count / 10).chain([count - 1]);

        for _ in 0..100 {
            simulate::trajectory(
                &planner.problem().reach,
                planner.nodes(),
                &obstacles,
                goal,
                &mut rng,
            );
        }

        for node in targets {
            simulate::trajectory(
                &planner.problem().reach,
                planner.nodes(),
                &obstacles,
                node,
                &mut rng,
            );
        }
    }
}

#[test]
fn cost_never_increases_and_ends_at_or_below_robust_rrt() {
    for seed in 0..3 {
        let (star, costs) = grown_slalom(seed);
        let mut rrt = planner(&SLALOM, [1.0, 1.0], [9.0, 9.0]);
        let rrt_cost = rrt
            .solve(ITERATIONS, &mut Pcg64Mcg::seed_from_u64(seed))
            .expect("the slalom is solvable")
            .cost;
        let star_cost = *costs.last().expect("solved");

        assert!(
            costs.windows(2).all(|pair| pair[1] <= pair[0]),
            "seed {seed}: the cost went up"
        );
        assert!(
            star_cost <= rrt_cost,
            "seed {seed}: RRT* {star_cost} above RRT {rrt_cost}"
        );
        assert_eq!(star.cost(star.goal_node().expect("solved")), star_cost);
    }
}

#[test]
fn same_seed_gives_the_same_tree() {
    let ((a, _), (b, _)) = (grown_slalom(5), grown_slalom(5));

    assert_eq!(a.nodes().len(), b.nodes().len());
    assert_eq!(a.path(), b.path());
    assert_eq!(a.stats(), b.stats());

    for (x, y) in a.nodes().iter().zip(b.nodes()) {
        assert_eq!(x.parent, y.parent);
        assert_eq!(x.control, y.control);
        assert_eq!(x.set.interval_hull(), y.set.interval_hull());
    }
}
