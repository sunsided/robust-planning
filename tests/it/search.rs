//! Reachability-guided search with the affine model on 2D worlds.

use polyzonotope::{Containment, Intersection, Polytope, SparsePolyZonotope, Zonotope};
use rand_pcg::Pcg64Mcg;
use robust_planning::{
    Cost, Obstacles, PolytopeGoal, Reach,
    search::{Config, Dedup, GoalRadius, Problem, ReachSearch},
};
use sampling_planning::rand_core::SeedableRng;

use crate::{
    affine::Affine,
    simulate,
    world::{Rect, SLALOM, SPLITS, interval_box, model, polytopes, square, wall_with_gap},
};

/// Walls around the 10 x 10 world, so that the lattice the search explores is finite.
const FRAME: [Rect; 4] = [
    ([-1.0, -1.0], [11.0, 0.0]),
    ([-1.0, 10.0], [11.0, 11.0]),
    ([-1.0, -1.0], [0.0, 11.0]),
    ([10.0, -1.0], [11.0, 11.0]),
];

/// `obstacles` inside [`FRAME`].
fn framed(obstacles: &[Rect]) -> Vec<Rect> {
    FRAME.iter().chain(obstacles).copied().collect()
}

/// The search type all tests here use.
type Search = ReachSearch<Affine<2>, 2>;

/// The 8-neighborhood at step `step`, which the test model reaches exactly per edge.
fn eight_neighborhood(step: f64) -> Vec<[f64; 2]> {
    let mut primitives = Vec::new();

    for dx in [-step, 0.0, step] {
        for dy in [-step, 0.0, step] {
            if dx != 0.0 || dy != 0.0 {
                primitives.push([dx, dy]);
            }
        }
    }

    primitives
}

/// A search in the framed world around `obstacles`, from a start box of half width 0.1 around
/// `start` to a goal box of half width 0.5 around `goal`, over the 8-neighborhood at 0.5.
fn search(obstacles: &[Rect], start: [f64; 2], goal: [f64; 2], config: Config) -> Search {
    let problem = Problem {
        reach: model(),
        obstacles: Obstacles::new(polytopes(&framed(obstacles)), SPLITS),
        start,
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&interval_box(&square(
            start, 0.1,
        )))),
        goal: PolytopeGoal::from_box(&interval_box(&square(goal, 0.5)), 8),
        primitives: eight_neighborhood(0.5),
        bounds: None,
    };

    ReachSearch::new(problem, config).expect("valid start and config")
}

/// A* settings: every goal node's nominal state is within the goal box's half diagonal of
/// its center, since the model's nominal state lies in its set.
fn astar() -> Config {
    Config {
        goal_radius: GoalRadius::Fixed(0.75),
        ..Config::default()
    }
}

/// Panics unless every edge of the search tree is proven disjoint from every obstacle,
/// re-propagated from the parent's set and checked with polyzonotope directly.
fn assert_every_edge_disjoint(search: &Search, obstacles: &[Rect]) {
    let obstacles = polytopes(&framed(obstacles));
    let mut cover = Vec::new();

    for node in &search.nodes()[1..] {
        let parent = &search.nodes()[node.parent.expect("non-root")];

        cover.clear();
        let end = search.problem().reach.propagate(
            &parent.set,
            node.control.as_ref().expect("non-root"),
            &mut cover,
        );
        assert!(end.is_some());
        assert_ne!(cover.len(), 0);

        for set in &cover {
            for obstacle in &obstacles {
                assert_eq!(set.intersection(obstacle, SPLITS), Intersection::Disjoint);
            }
        }
    }
}

#[test]
fn open_world_path_is_the_lattice_optimum() {
    for (goal, expected) in [
        ([3.0, 1.0], 2.0),
        ([1.0, 4.0], 3.0),
        ([3.0, 3.0], 2.0 * 2.0_f64.sqrt()),
        ([4.0, 2.0], 2.0 + 2.0_f64.sqrt()),
    ] {
        for config in [astar(), Config::default()] {
            let mut search = search(&[], [1.0, 1.0], goal, config);
            let path = search.solve().expect("the open world is solvable");

            assert!(
                (path.cost - expected).abs() < 1e-9,
                "goal {goal:?}: cost {} instead of {expected}",
                path.cost
            );
        }
    }
}

#[test]
fn astar_matches_uniform_cost_on_the_slalom_and_expands_less() {
    let mut informed = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], astar());
    let mut uniform = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], Config::default());
    let a = informed.solve().expect("the slalom is solvable");
    let u = uniform.solve().expect("the slalom is solvable");

    assert!((a.cost - u.cost).abs() < 1e-9, "{} vs {}", a.cost, u.cost);
    assert!(informed.expansions() <= uniform.expansions());
    assert_every_edge_disjoint(&informed, &SLALOM);
    assert_every_edge_disjoint(&uniform, &SLALOM);
}

#[test]
fn monte_carlo_trajectories_along_the_slalom_path_never_hit_obstacles() {
    let mut search = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], astar());
    search.solve().expect("the slalom is solvable");
    let goal = search.goal_node().expect("solved");
    let obstacles = polytopes(&framed(&SLALOM));
    let mut rng = Pcg64Mcg::seed_from_u64(17);

    for _ in 0..50 {
        simulate::trajectory(
            &search.problem().reach,
            search.nodes(),
            &obstacles,
            goal,
            &mut rng,
        );
    }
}

#[test]
fn narrow_passage_exhausts_the_search() {
    let narrow = wall_with_gap(0.2);
    let mut blocked = search(&narrow, [1.0, 5.0], [9.0, 5.0], astar());

    assert!(blocked.solve().is_none());
    assert!(blocked.is_exhausted(), "the reachable lattice is finite");
    assert!(blocked.nodes().iter().all(|n| n.nominal[0] < 4.5));
    assert_every_edge_disjoint(&blocked, &narrow);

    let mut wide = search(&wall_with_gap(1.0), [1.0, 5.0], [9.0, 5.0], astar());
    assert!(wide.solve().is_some(), "the same search passes a wide gap");
}

#[test]
fn no_kept_node_is_dominated_by_an_older_one_in_its_cell() {
    let mut search = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], astar());
    search.solve();
    let nodes = search.nodes();
    let cell = |x: &[f64; 2]| x.map(|v| (v * 1e6).round());

    for (b, newer) in nodes.iter().enumerate() {
        for (a, older) in nodes[..b].iter().enumerate() {
            #[expect(
                clippy::float_cmp,
                reason = "cells are rounded integers, compared exactly as the search does"
            )]
            let other_cell = cell(&older.nominal) != cell(&newer.nominal);

            if other_cell {
                continue;
            }

            let (ha, hb) = (older.set.interval_hull(), newer.set.interval_hull());
            let within =
                ha.0.iter()
                    .zip(&hb.0)
                    .all(|(i, o)| o.lo <= i.lo && i.hi <= o.hi);

            assert!(
                !(search.cost(a) <= search.cost(b) && within),
                "node {b} is dominated by node {a}"
            );
        }
    }
}

#[test]
fn budget_stop_can_be_resumed() {
    let config = Config {
        max_expansions: 5,
        ..astar()
    };
    let mut search = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], config);

    assert!(search.solve().is_none());
    assert_eq!(search.expansions(), 5);
    assert!(!search.is_exhausted());

    while search.solve().is_none() {
        assert!(!search.is_exhausted(), "the slalom is solvable");
    }

    assert!(search.goal_node().is_some());
}

#[test]
fn width_weighted_cost_is_optimal_with_and_without_the_heuristic() {
    let weighted = Cost::WidthWeighted { weight: 2.0 };
    let solve = |config: Config| {
        let mut search = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], config);
        let path = search.solve().expect("the slalom is solvable");

        (search, path)
    };
    let (search, astar_path) = solve(Config {
        cost: weighted,
        ..astar()
    });
    let (_, uniform_path) = solve(Config {
        cost: weighted,
        ..Config::default()
    });
    let (_, length_path) = solve(astar());
    let (_, zero_path) = solve(Config {
        cost: Cost::WidthWeighted { weight: 0.0 },
        ..astar()
    });

    assert!((astar_path.cost - uniform_path.cost).abs() < 1e-9);
    assert!(astar_path.cost > length_path.cost, "widths are charged");
    assert_eq!(
        zero_path.cost, length_path.cost,
        "weight 0 prices like length"
    );

    let goal = search.goal_node().expect("solved");
    let ids = search.branch(goal);
    let recomputed: f64 = ids
        .windows(2)
        .map(|pair| {
            let (from, to) = (&search.nodes()[pair[0]], &search.nodes()[pair[1]]);
            let length = ((to.nominal[0] - from.nominal[0]).powi(2)
                + (to.nominal[1] - from.nominal[1]).powi(2))
            .sqrt();

            weighted.edge(length, &to.set)
        })
        .sum();
    assert!((recomputed - astar_path.cost).abs() < 1e-9);
}

/// A search without framing walls around `obstacles`, over the 8-neighborhood at 0.5,
/// with nominal states bounded by `bounds`.
fn bounded(
    obstacles: &[Rect],
    goal: [f64; 2],
    bounds: Option<([f64; 2], [f64; 2])>,
    config: Config,
) -> Result<Search, robust_planning::search::Error> {
    let problem = Problem {
        reach: model(),
        obstacles: Obstacles::new(polytopes(obstacles), SPLITS),
        start: [1.0, 1.0],
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&interval_box(&square(
            [1.0, 1.0],
            0.1,
        )))),
        goal: PolytopeGoal::from_box(&interval_box(&square(goal, 0.5)), 8),
        primitives: eight_neighborhood(0.5),
        bounds,
    };

    ReachSearch::new(problem, config)
}

#[test]
fn workspace_bounds_keep_an_unframed_search_finite() {
    let bounds = ([0.0, 0.0], [4.0, 4.0]);
    let mut search = bounded(&[], [9.0, 9.0], Some(bounds), Config::default()).expect("valid");

    assert!(search.solve().is_none(), "the goal lies outside the bounds");
    assert!(search.is_exhausted(), "the bounded lattice is finite");
    assert!(
        search
            .nodes()
            .iter()
            .all(|n| n.nominal.iter().all(|x| (0.0..=4.0).contains(x)))
    );

    let mut reachable = bounded(&[], [3.0, 3.0], Some(bounds), Config::default()).expect("valid");
    let path = reachable.solve().expect("the goal lies inside the bounds");
    assert!((path.cost - 2.0 * 2.0f64.sqrt()).abs() < 1e-9);
}

#[test]
fn invalid_bounds_are_rejected() {
    use robust_planning::search::Error;

    let invalid = |bounds| bounded(&[], [3.0, 3.0], Some(bounds), Config::default()).err();

    assert_eq!(
        invalid(([2.0, 0.0], [4.0, 4.0])),
        Some(Error::InvalidBounds)
    );
    assert_eq!(
        invalid(([0.0, 4.0], [4.0, 0.0])),
        Some(Error::InvalidBounds)
    );
    assert_eq!(
        invalid(([f64::NAN, 0.0], [4.0, 4.0])),
        Some(Error::InvalidBounds)
    );
}

#[test]
fn goal_radius_from_the_goal_matches_uniform_cost_and_expands_less() {
    let derived = Config {
        goal_radius: GoalRadius::FromGoal,
        ..Config::default()
    };
    let mut informed = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], derived);
    let mut uniform = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], Config::default());
    let a = informed.solve().expect("the slalom is solvable");
    let u = uniform.solve().expect("the slalom is solvable");

    assert!((a.cost - u.cost).abs() < 1e-9, "{} vs {}", a.cost, u.cost);
    assert!(informed.expansions() < uniform.expansions());
}

#[test]
fn goal_radius_from_a_goal_without_one_is_an_error() {
    use polyzonotope::{Halfspace, Polytope};
    use robust_planning::search::Error;

    let facet = |a: f64, b: f64, offset: f64| Halfspace {
        normal: nalgebra::DVector::from_column_slice(&[a, b]),
        offset,
    };
    let triangle = Polytope {
        facets: vec![
            facet(-1.0, 0.0, -8.0),
            facet(0.0, -1.0, -8.0),
            facet(1.0, 1.0, 19.0),
        ],
    };
    let problem = Problem {
        reach: model(),
        obstacles: Obstacles::new(Vec::new(), SPLITS),
        start: [1.0, 1.0],
        start_set: SparsePolyZonotope::from_zonotope(&Zonotope::from_box(&interval_box(&square(
            [1.0, 1.0],
            0.1,
        )))),
        goal: PolytopeGoal::new(triangle, [9.0, 9.0], 8),
        primitives: eight_neighborhood(0.5),
        bounds: None,
    };
    let config = Config {
        goal_radius: GoalRadius::FromGoal,
        ..Config::default()
    };

    assert_eq!(
        ReachSearch::new(problem, config).err(),
        Some(Error::GoalRadiusUnavailable)
    );
}

#[test]
fn pruning_superseded_subtrees_stays_sound() {
    // With the consistent heuristic an expanded node is superseded only on an exact cost
    // tie, so on the slalom nothing is pruned and the result matches; the unit tests in
    // `search.rs` cover the pruning itself.
    for base in [astar(), Config::default()] {
        let pruning = Config {
            prune_superseded_subtrees: true,
            ..base
        };
        let mut pruned = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], pruning);
        let mut plain = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], base);
        let p = pruned.solve().expect("the slalom is solvable with pruning");
        let q = plain.solve().expect("the slalom is solvable");

        assert_eq!(p, q);
        assert!(pruned.expansions() <= plain.expansions());
        assert_every_edge_disjoint(&pruned, &SLALOM);
    }

    let pruning = Config {
        prune_superseded_subtrees: true,
        ..astar()
    };
    let mut pruned = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], pruning);
    pruned.solve().expect("the slalom is solvable with pruning");
    let goal = pruned.goal_node().expect("solved");
    let obstacles = polytopes(&framed(&SLALOM));
    let mut rng = Pcg64Mcg::seed_from_u64(23);

    for _ in 0..50 {
        simulate::trajectory(
            &pruned.problem().reach,
            pruned.nodes(),
            &obstacles,
            goal,
            &mut rng,
        );
    }
}

#[test]
fn proven_dedup_finds_the_same_optimum_and_stays_sound() {
    // A small open world: proven deduplication keeps many more nodes (on the slalom about
    // ten times the expansions of the hull test), which is too slow for a debug test there.
    let proven = Config {
        dedup: Dedup::Proven { tolerance: 0.0 },
        ..astar()
    };
    let mut strict = search(&[], [1.0, 1.0], [4.0, 2.0], proven);
    let mut hull = search(&[], [1.0, 1.0], [4.0, 2.0], astar());
    let s = strict.solve().expect("the open world is solvable");
    let h = hull.solve().expect("the open world is solvable");

    assert!((s.cost - (2.0 + 2.0f64.sqrt())).abs() < 1e-9);
    assert!((s.cost - h.cost).abs() < 1e-9);
    assert!(strict.nodes().len() >= hull.nodes().len());
    assert_every_edge_disjoint(&strict, &[]);

    let default_tolerance = Config {
        dedup: Dedup::proven(),
        ..astar()
    };
    let mut tolerant = search(&[], [1.0, 1.0], [4.0, 2.0], default_tolerance);
    let t = tolerant.solve().expect("the open world is solvable");

    assert!((t.cost - h.cost).abs() < 1e-9);
    assert!(
        tolerant.nodes().len() <= strict.nodes().len(),
        "the tolerance keeps no more nodes here"
    );
    assert_every_edge_disjoint(&tolerant, &[]);
}

#[test]
#[ignore = "slow in debug builds; run with --release -- --ignored --nocapture"]
fn proven_dedup_expansions_on_the_slalom() {
    let hull = {
        let mut search = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], astar());
        let path = search.solve().expect("the slalom is solvable");

        (search.expansions(), path.cost)
    };
    eprintln!("hull: expansions {} cost {}", hull.0, hull.1);

    for tolerance in [0.0, 1e-12, 1e-9, 1e-6] {
        let config = Config {
            dedup: Dedup::Proven { tolerance },
            ..astar()
        };
        let mut search = search(&SLALOM, [1.0, 1.0], [9.0, 9.0], config);
        let path = search.solve().expect("the slalom is solvable");
        eprintln!(
            "proven tolerance {tolerance:e}: expansions {} cost {}",
            search.expansions(),
            path.cost
        );

        assert!((path.cost - hull.1).abs() < 1e-9);
        assert_every_edge_disjoint(&search, &SLALOM);
    }
}

#[test]
fn bounded_sets_stay_proven_inside_the_workspace() {
    let bounds = ([0.0, 0.0], [4.0, 4.0]);
    let sets = Config {
        bounds_contain_sets: true,
        ..Config::default()
    };
    let mut nominal = bounded(&[], [9.0, 9.0], Some(bounds), Config::default()).expect("valid");
    let mut whole = bounded(&[], [9.0, 9.0], Some(bounds), sets).expect("valid");
    nominal.solve();
    whole.solve();

    assert!(whole.is_exhausted());
    assert!(
        whole.nodes().len() < nominal.nodes().len(),
        "nodes on the border have sets that stick out"
    );

    let workspace = Polytope::from_box(&interval_box(&bounds));
    let mut cover = Vec::new();

    for node in &whole.nodes()[1..] {
        let parent = &whole.nodes()[node.parent.expect("non-root")];
        cover.clear();
        let end = whole
            .problem()
            .reach
            .propagate(
                &parent.set,
                node.control.as_ref().expect("non-root"),
                &mut cover,
            )
            .expect("every accepted edge has an end set");

        for set in cover.iter().chain([&end]) {
            assert_eq!(set.containment(&workspace, SPLITS), Containment::Inside);
        }
    }

    let mut reachable = bounded(&[], [3.0, 3.0], Some(bounds), sets).expect("valid");
    assert!(
        reachable.solve().is_some(),
        "the goal box lies inside the bounds"
    );
}

#[test]
fn a_start_set_outside_the_workspace_is_rejected_only_when_sets_are_bounded() {
    use robust_planning::search::Error;

    // The start set reaches down to 0.9; the nominal start 1.0 is inside.
    let bounds = Some(([0.95, 0.95], [4.0, 4.0]));
    let sets = Config {
        bounds_contain_sets: true,
        ..Config::default()
    };

    assert!(bounded(&[], [3.0, 3.0], bounds, Config::default()).is_ok());
    assert_eq!(
        bounded(&[], [3.0, 3.0], bounds, sets).err(),
        Some(Error::InvalidBounds)
    );
}
