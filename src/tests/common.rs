use coordinate::Coordinate;
use math_util::Feq;
use rstar::{AABB, RTreeObject, PointDistance};

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
pub struct MonoMBR {
    ll: Pt,
    ur: Pt,
    i: usize,
    j: usize,
}

impl MonoMBR {
   pub fn new(a: Pt, b: Pt, i: usize, j: usize) -> MonoMBR {
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




