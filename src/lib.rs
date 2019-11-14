use rstar::{
    RTreeObject,
    RTree as Index,
    Envelope,
    PointDistance,
    AABB,
};
use bbox_2d::MBR;


#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
feature = "serde", serde(bound(
serialize = "T: Serialize, T::Envelope: Serialize",
deserialize = "T: Deserialize<'de>, T::Envelope: Deserialize<'de>")))
]
pub struct RTree<T> where T: RTreeObject {
    index: Index<T>,
}

impl<T> RTree<T> where T: RTreeObject + PointDistance {
    pub fn new() -> Self {
        RTree { index: Index::new() }
    }

    pub fn load(item: Vec<T>) -> Self {
        RTree { index: Index::bulk_load_with_params(item) }
    }


    pub fn insert(&mut self, item: T) {
        self.index.insert(item)
    }


    pub fn size(&self) -> usize {
        self.index.size()
    }

//    pub fn search(&self, envelope: &T::Envelope) -> Vec<&T> {
//        //        let box  = rstar::object::RTreeObject
////        let query_envelope = AABB::from_corners(
////            envelope.ll.as_array(),
////            envelope.ur.as_array(),
////        );
//
//
//        let elements = self.index.locate_in_envelope_intersecting(
//            &envelope
//        );
//        let mut results = vec![];
//        for o in elements.into_iter() {
//            results.push(o)
//        }
//        results
//    }

    pub fn nearest_neighbor(&self, query_pt: &<T::Envelope as Envelope>::Point) -> Option<&T> {
        self.index.nearest_neighbor(query_pt)
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use bbox_2d::{MBR, BBox};
    use geom_2d::{pt, Point};

    #[test]
    fn it_works() {
        let items = vec![
            MBR::new(pt!(0., 0.), pt!( 1., 1.)),
            MBR::new(pt!(1., 1.), pt!( 3., 3.)),
            MBR::new(pt!(4., 2.), pt!( 7.0, 3.0))
        ];
        let mut tree = RTree::load(items);

//        let query = MBR::new(pt!(1.5, 0.5), pt!( 4.0, 2.5));
//        let res = tree.search(&query);
//        if let Some(r) = res {
//            println!("intersecting {:?}", r);
//        }
    }
}
