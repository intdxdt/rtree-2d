mod common;

use coordinate::Coordinate;
use math_util::Feq;
use rstar::{AABB, Envelope};
use common::{MonoMBR, Pt};
use super::{*};

#[test]
fn test_rtree() {
    let items = vec![
        MonoMBR::new(Pt { x: 0., y: 0. }, Pt { x: 1., y: 1. }, 0, 3),
        MonoMBR::new(Pt { x: 1., y: 1. }, Pt { x: 2., y: 2. }, 3, 7),
        MonoMBR::new(Pt { x: 4., y: 2. }, Pt { x: 7.0, y: 3.0 }, 7, 11),
    ];
    let mut tree = RTree::load(items);
    assert_eq!(tree.size(), 3);
    let dist = std::f64::MAX;
    let query = MonoMBR::new(Pt { x: 0., y: 0. }, Pt { x: 1., y: 1. }, 0, 3);
    tree.knn_min_dist(
        &query,
        |query: &MonoMBR, item: &MonoMBR| {
            (3.4, 6.7)
        },
        |o| {
            o.distance > dist || dist == 0.0 //add to neibs, stop
        },
    );

    println!("{}", MonoMBR::from(tree.root().envelope()).wkt());

    let query = MonoMBR::new(Pt { x: 2.5, y: 0.5 }, Pt { x: 4.0, y: 2.5 }, 0, 9);
    let res = tree.search(&query.envelope());
    for r in res.into_iter() {
        assert!(tree.contains(r));
    }
    let at = MonoMBR::new(Pt { x: 1., y: 1. }, Pt { x: 2., y: 2. }, 3, 7);
    let res = tree.remove(&at);
    assert_eq!(tree.size(), 2);
    assert!(!tree.contains(&at));

    match res {
        Some(v) => {
            print!("rm = {}", v.wkt())
        }
        None => println!("None!")
    }
    println!("tree size = {}", tree.size());
    tree.each(|v| println!("{}", v.wkt()));
    let rt = tree.rtree();
    assert_eq!(rt.size(), 2);
}
