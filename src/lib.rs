mod kobj;

pub use rstar::{
    RTreeObject,
    RTree as Index,
    Envelope,
    Point,
    PointDistance,
    ParentNode,
    AABB,
};
use std::collections::BinaryHeap;
use kobj::KObj;
use bbox_2d::MBR;
use math_util::{num, NumCast};
use rstar::RTreeNode;

#[derive(Clone, Debug)]
pub struct RTree<T> where T: RTreeObject {
    index: Index<T>,
}

impl<T> RTree<T> where T: RTreeObject + PartialEq {
    pub fn remove(&mut self, item: &T) -> Option<T> {
        self.index.remove(item)
    }

    pub fn contains(&self, item: &T) -> bool {
        self.index.contains(item)
    }
}

impl<T> RTree<T> where T: RTreeObject + PointDistance {
    pub fn nearest_neighbor(&self, query_pt: &<T::Envelope as Envelope>::Point) -> Option<&T> {
        self.index.nearest_neighbor(query_pt)
    }

    pub fn locate_within_distance(
        &self,
        query_pt: <T::Envelope as Envelope>::Point,
        sqr_radius: <<T::Envelope as Envelope>::Point as rstar::Point>::Scalar,
    ) -> Vec<&T> {
        let mut results = vec![];
        for o in self.index.locate_within_distance(query_pt, sqr_radius) {
            results.push(o);
        }
        results
    }

    pub fn remove_at_point(&mut self, pt: &<T::Envelope as Envelope>::Point) -> Option<T> {
        self.index.remove_at_point(pt)
    }
}

impl<T> RTree<T> where T: RTreeObject + Clone {
    pub fn new() -> Self {
        RTree { index: Index::new() }
    }

    pub fn load(items: Vec<T>) -> Self {
        RTree { index: Index::bulk_load(items) }
    }

    pub fn root(&self) -> &ParentNode<T> {
        self.index.root()
    }

    pub fn each(&self, func: fn(&T)) {
        for item in self.index.iter() {
            func(item);
        }
    }

    pub fn rtree(&self) -> Index<T> {
        self.index.clone()
    }

    pub fn insert(&mut self, item: T) {
        self.index.insert(item)
    }


    pub fn size(&self) -> usize {
        self.index.size()
    }

    pub fn is_empty(&self) -> bool {
        self.index.size() == 0
    }

    pub fn clear(&mut self) {
        self.index = Index::new()
    }

    pub fn search(&self, envelope: &T::Envelope) -> Vec<&T> {
        let elements = self.index.locate_in_envelope_intersecting(
            envelope
        );
        let mut results = vec![];
        for o in elements.into_iter() {
            results.push(o)
        }
        results
    }

    pub fn knn_min_dist(
        &self, query: &T,
        distScore: fn(query: &T, dbitem: &T) -> (f64, f64),
        predicate: impl Fn(KObj) -> bool,
    ) {
        let as_mbr = |envelope: &T::Envelope| {
            let ll = envelope.lower_left();
            let ur = envelope.upper_right();
            MBR::new(
                num::cast(ll.nth(0)).unwrap(),
                num::cast(ll.nth(1)).unwrap(),
                num::cast(ur.nth(0)).unwrap(),
                num::cast(ur.nth(1)).unwrap(),
            )
        };

        let query_box = as_mbr(&query.envelope());
        let nd = Some(self.index.root());
        let children = nd.unwrap().children();
        let stop: bool = false;
        let mut queue = BinaryHeap::new();
        let mindist = std::f64::MAX;


        'outer: while !stop && nd.is_some() {
            for child in nd.unwrap().children().iter() {
                let child_box = as_mbr(&child.envelope());
                let box_dist = child_box.distance(&query_box);
                if box_dist < mindist {
                    let mut o = KObj {
                        distance: 0f64,
                        is_item: child.is_leaf(),
                        mbr: child_box,
                    };
                    match child {
                        RTreeNode::Leaf(ref item) => {
                            let (o_dist, mindist) = distScore(query, item);
                            o.distance = o_dist;
                        }
                        _ => {}
                    }
                    queue.push(o)
                }
            }
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use coordinate::Coordinate;
    use math_util::Feq;
    use rstar::AABB;

    #[derive(Copy, Clone, PartialOrd, Debug)]
    pub struct Pt {
        pub x: f64,
        pub y: f64,
    }

    impl Pt {
        pub fn equals(&self, other: &Pt) -> bool {
            self.x.feq(other.x) & &self.y.feq(other.y)
        }
    }

    impl Coordinate for Pt {
        type Scalar = f64;
        const DIM: usize = 2;
        ///Type Constructor : Generator over dimensions
        fn gen(value: impl Fn(usize) -> Self::Scalar) -> Self {
            Pt {
                x: value(0),
                y: value(1),
            }
        }
        ///Value in ith dimension
        fn val(&self, i: usize) -> Self::Scalar {
            match i {
                0 => self.x,
                1 => self.y,
                _ => unreachable!(),
            }
        }
        ///Mutable value in ith dimension
        fn val_mut(&mut self, i: usize) -> &mut Self::Scalar {
            match i {
                0 => &mut self.x,
                1 => &mut self.y,
                _ => unreachable!(),
            }
        }
    }

    impl PartialEq for Pt {
        fn eq(&self, other: &Self) -> bool {
            self.equals(other)
        }
    }

    impl rstar::Point for Pt {
        type Scalar = f64;
        const DIMENSIONS: usize = 2;
        fn generate(generator: impl Fn(usize) -> Self::Scalar) -> Self {
            Pt::gen(generator)
        }
        fn nth(&self, index: usize) -> Self::Scalar {
            self.val(index)
        }
        fn nth_mut(&mut self, index: usize) -> &mut Self::Scalar {
            self.val_mut(index)
        }
    }

    #[derive(Clone, Debug)]
    struct MonoMBR {
        ll: Pt,
        ur: Pt,
        i: usize,
        j: usize,
    }

    impl MonoMBR {
        fn new(a: Pt, b: Pt, i: usize, j: usize) -> MonoMBR {
            MonoMBR {
                ll: a.min_of_bounds(&b),
                ur: a.max_of_bounds(&b),
                i,
                j,
            }
        }

        pub fn equals(&self, other: &Self) -> bool {
            self.ll.equals(&other.ll) & &self.ur.equals(&other.ur)
        }

        pub fn intersects(&self, other: &Self) -> bool {
            //not disjoint
            !(other.ll.x > self.ur.x
                || other.ur.x < self.ll.x
                || other.ll.y > self.ur.y
                || other.ur.y < self.ll.y)
        }
        pub fn distance_dxdy(&self, other: &Self) -> (f64, f64) {
            let mut dx = 0.0;
            let mut dy = 0.0;

            // find closest edge by x
            if self.ur.x < other.ll.x {
                dx = other.ll.x - self.ur.x
            } else if self.ll.x > other.ur.x {
                dx = self.ll.x - other.ur.x
            }

            // find closest edge by y
            if self.ur.y < other.ll.y {
                dy = other.ll.y - self.ur.y
            } else if self.ll.y > other.ur.y {
                dy = self.ll.y - other.ur.y
            }

            (dx, dy)
        }
        pub fn distance_square(&self, other: &Self) -> f64 {
            if self.intersects(other) {
                return 0.0;
            }
            let (dx, dy) = self.distance_dxdy(other);
            (dx * dx) + (dy * dy)
        }
        pub fn wkt(&self) -> String {
            format!(
                "POLYGON (({lx} {ly},{lx} {uy},{ux} {uy},{ux} {ly},{lx} {ly}))",
                lx = self.ll.x,
                ly = self.ll.y,
                ux = self.ur.x,
                uy = self.ur.y
            )
        }
    }

    impl PartialEq for MonoMBR {
        fn eq(&self, other: &Self) -> bool {
            self.equals(other)
        }
    }

    impl From<AABB<Pt>> for MonoMBR {
        fn from(aab: AABB<Pt>) -> Self {
            MonoMBR::new(aab.lower(), aab.upper(), 0, 0)
        }
    }


    impl RTreeObject for MonoMBR {
        type Envelope = AABB<Pt>;
        fn envelope(&self) -> Self::Envelope {
            AABB::from_corners(self.ll, self.ur)
        }
    }

    impl PointDistance for MonoMBR {
        fn distance_2(&self, pt: &Pt) -> f64 {
            self.distance_square(&MonoMBR::new(*pt, *pt, 0, 0))
        }
    }

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
            |o: KObj| {
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
}
