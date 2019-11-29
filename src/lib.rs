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
        fn_dist_score: fn(query: &T, dbitem: &T) -> (f64, f64),
        fn_predicate: impl Fn(KObj) -> bool,
    ) {
        if self.is_empty() {
            return;
        }

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
        let mut parents = vec![Some(self.index.root())];
        let mut nd = parents[0];
        let mut stop: bool = false;
        let mut queue = BinaryHeap::new();
        let mindist = std::f64::MAX;
        let null_idx = std::usize::MAX;

        'outer: while !stop && nd.is_some() {
            for child in nd.unwrap().children().iter() {
                let child_box = as_mbr(&child.envelope());
                let box_dist = child_box.distance(&query_box);
                if box_dist < mindist {
                    let mut o = KObj {
                        distance: 0f64,
                        is_item: child.is_leaf(),
                        mbr: child_box,
                        node: null_idx,
                    };
                    match child {
                        RTreeNode::Leaf(ref item) => {
                            let (o_dist, mindist) = fn_dist_score(query, item);
                            o.distance = o_dist;
                        }
                        RTreeNode::Parent(ref p)=> {
                            o.node = parents.len();
                            parents.push(Some(p));
                        }
                    }
                    queue.push(o)
                }
            }

            while !queue.is_empty() && queue.peek().unwrap().is_item {
                let candidate = queue.pop().unwrap();
                stop = fn_predicate(candidate);
                if stop {
                    break;
                }
            }

            if !stop {
                let q = queue.pop();
                if q.is_none() {
                    nd = None
                } else {
                    nd = parents[q.unwrap().node]
                }
            }
        }
    }
}


#[cfg(test)]
mod tests;
