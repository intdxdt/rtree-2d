use rstar::{RTreeObject, RTree as Index, Envelope, PointDistance};


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
            MBR::new(pt!(480., 400.), pt!( 550., 484.)),
            MBR::new(pt!(673., 336.), pt!( 773., 486.)),
            MBR::new(pt!(285., 277.), pt!( 385.0, 377.0))
        ];
        let mut tree = RTree::load(items);

        let query = [526., 324.].into();
        let res = tree.nearest_neighbor(&query);
        if let Some(r) = res {
            println!("intersecting {:?}", r);
        }
    }
}
