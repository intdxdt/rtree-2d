use std::cmp::Ordering;
use std::collections::BinaryHeap;
use math_util::Feq;
use bbox_2d::MBR;
use rstar::{RTreeObject, RTreeNode, ParentNode};


///
///Min - Heap KObj
///
#[derive(Clone, Copy, Debug)]
pub struct KObj  {
    pub distance: f64,
    pub is_item: bool,
    pub mbr: MBR,
    pub node:usize,
}

impl Eq for KObj{}

impl PartialEq for KObj  {
    fn eq(&self, other: &Self) -> bool {
        self.distance.feq(other.distance)
    }
}

impl PartialOrd for KObj {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        other.distance.partial_cmp(&self.distance)
    }
}

impl Ord for KObj where {
    fn cmp(&self, other: &KObj) -> Ordering {
        self.partial_cmp(other).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_min_heap() {
        let mut minheap = BinaryHeap::new();
        minheap.push(KObj { distance: 42.0, is_item: false, mbr: MBR::new_default(), node: 0 });
        minheap.push(KObj { distance: 0.42, is_item: false, mbr: MBR::new_default(), node: 0 });
        minheap.push(KObj { distance: 1.0, is_item: false, mbr: MBR::new_default(), node:  0 });
        minheap.push(KObj { distance: 0.3, is_item: false, mbr: MBR::new_default(), node:  0 });
        if let Some(o) = minheap.pop() {
            println!("{:?}", o);
        }
        if let Some(o) = minheap.pop() {
            println!("{:?}", o);
        }
        if let Some(o) = minheap.pop() {
            println!("{:?}", o);
        }
        if let Some(o) = minheap.pop() {
            println!("{:?}", o);
        }
//    if let Some(Obj(root)) = minheap.pop() {
//        println!("{:?}", root);
//    }
    }
}