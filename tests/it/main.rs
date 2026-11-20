//! Integration tests: Robust-RRT and the reachability checker on 2D worlds.

mod affine;
mod goal;
mod search;
mod simulate;
mod star;
mod steering;
mod unknown;
mod witness;
mod world;

use polyzonotope::Intersection;
use rand_pcg::Pcg64Mcg;
use robust_planning::{Reach, ReachChecker};
use sampling_planning::{
    Euclidean, GoalRegion, Problem, StraightLine, UniformBox,
    rand_core::SeedableRng,
    trees::{Rrt, RrtConfig},
};
use world::{PointChecker, SLALOM, SPLITS, model, planner, polytopes, wall_with_gap};

/// Iteration budget for every planner run.
const ITERATIONS: usize = 3000;

/// A Robust-RRT on the slalom, solved with `seed`.
fn solved_slalom(seed: u64) -> world::Planner {
    let mut planner = planner(&SLALOM, [1.0, 1.0], [9.0, 9.0]);
    let path = planner.solve(ITERATIONS, &mut Pcg64Mcg::seed_from_u64(seed));

    assert!(path.is_some(), "seed {seed}: the slalom is solvable");
    assert!(
        planner.nodes().len() > 20,
        "seed {seed}: the tree is trivial"
    );

    planner
}

#[test]
fn every_accepted_edge_is_proven_disjoint() {
    let obstacles = polytopes(&SLALOM);

    for seed in 0..3 {
        let planner = solved_slalom(seed);
        let reach = &planner.problem().reach;
        let mut cover = Vec::new();

        for node in &planner.nodes()[1..] {
            let parent = &planner.nodes()[node.parent.expect("non-root")];
            let control = node.control.as_ref().expect("non-root");

            cover.clear();
            reach.propagate(&parent.set, control, &mut cover);
            assert_ne!(cover.len(), 0, "every edge has a cover");

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
fn monte_carlo_trajectories_never_hit_obstacles() {
    let obstacles = polytopes(&SLALOM);
    let mut rng = Pcg64Mcg::seed_from_u64(99);

    for seed in 0..2 {
        let planner = solved_slalom(seed);
        let goal = planner.goal_node().expect("solved");
        let count = planner.nodes().len();
        let targets = (0..10).map(|k| k * count / 10).chain([goal, count - 1]);

        for node in targets {
            for _ in 0..30 {
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
}

#[test]
fn same_seed_gives_the_same_tree() {
    let (a, b) = (solved_slalom(7), solved_slalom(7));

    assert_eq!(a.nodes().len(), b.nodes().len());
    assert_eq!(a.goal_node(), b.goal_node());
    assert_eq!(a.path(), b.path());

    for (x, y) in a.nodes().iter().zip(b.nodes()) {
        assert_eq!(x.nominal, y.nominal);
        assert_eq!(x.parent, y.parent);
        assert_eq!(x.control, y.control);
        assert_eq!(x.set.interval_hull(), y.set.interval_hull());
    }
}

#[test]
fn robust_planner_rejects_a_passage_narrower_than_its_set() {
    let (start, goal) = ([1.0, 5.0], [9.0, 5.0]);
    let narrow = wall_with_gap(0.2);

    let mut point =
        Rrt::new(point_problem(&narrow, start, goal), RrtConfig::default()).expect("valid start");
    assert!(
        point
            .solve(ITERATIONS, &mut Pcg64Mcg::seed_from_u64(1))
            .is_some(),
        "a point robot fits through the gap"
    );

    let mut robust = planner(&narrow, start, goal);
    assert!(
        robust
            .solve(ITERATIONS, &mut Pcg64Mcg::seed_from_u64(1))
            .is_none()
    );
    assert!(
        robust.nodes().iter().all(|n| n.nominal[0] < 4.5),
        "no robust node gets past the wall"
    );

    let mut wide = planner(&wall_with_gap(1.0), start, goal);
    assert!(
        wide.solve(ITERATIONS, &mut Pcg64Mcg::seed_from_u64(1))
            .is_some(),
        "the same planner passes a wide gap"
    );
}

#[test]
fn reach_checker_rejects_the_narrow_passage_in_sampling_rrt() {
    let (start, goal) = ([1.0, 5.0], [9.0, 5.0]);
    let solve = |gap: f64| {
        let checker = ReachChecker::new(
            robust_planning::Obstacles::new(polytopes(&wall_with_gap(gap)), SPLITS),
            model(),
            [0.2, 0.2],
        );
        let problem = world_problem(checker, start, goal);
        let config = RrtConfig { goal_bias: 0.2 };
        let mut rrt = Rrt::new(problem, config).expect("valid start");

        rrt.solve(ITERATIONS, &mut Pcg64Mcg::seed_from_u64(3))
    };

    assert!(
        solve(0.2).is_none(),
        "the tube does not fit through the gap"
    );
    assert!(solve(1.0).is_some(), "the tube fits through a wide gap");
}

/// A `sampling-planning` problem for a point robot in the 10 x 10 world.
fn point_problem(
    obstacles: &[world::Rect],
    start: [f64; 2],
    goal: [f64; 2],
) -> Problem<Euclidean<2>, UniformBox<2>, PointChecker, StraightLine> {
    let checker = PointChecker {
        obstacles: polytopes(obstacles),
    };

    world_problem(checker, start, goal)
}

/// A `sampling-planning` problem in the 10 x 10 world with the given checker.
///
/// Steps are 0.9 long, so the test model (at most 1.0 per edge) always reaches the target.
fn world_problem<C>(
    checker: C,
    start: [f64; 2],
    goal: [f64; 2],
) -> Problem<Euclidean<2>, UniformBox<2>, C, StraightLine> {
    Problem {
        space: Euclidean::<2>,
        sampler: UniformBox::new([0.0, 0.0], [10.0, 10.0]),
        checker,
        steering: StraightLine::new(0.9),
        start,
        goal: GoalRegion::new(goal, 0.3),
    }
}
