use coordinate::Coordinate;
use math_util::{num, NumCast,Feq, EPSILON};
use rstar::{AABB, RTreeObject, PointDistance};

#[derive(Copy, Clone, PartialOrd, Debug)]
pub struct Pt {
    pub x: f64,
    pub y: f64,
}

pub struct Pts {
    pub pts : Vec<Pt>
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

impl<T> From<[T; 2]> for Pt where T: NumCast + Copy {
    fn from(array: [T; 2]) -> Self {
        Pt { x: num::cast(array[0]).unwrap(), y: num::cast(array[1]).unwrap() }
    }
}

impl<T> From<&[T; 2]> for Pt where T: NumCast + Copy {
    fn from(array: &[T; 2]) -> Self {
        Pt { x: num::cast(array[0]).unwrap(), y: num::cast(array[1]).unwrap() }
    }
}

impl<T> From<Vec<[T; 2]>> for Pts where T: NumCast + Copy {
    fn from(items: Vec<[T; 2]>) -> Self {
        let mut pts = Vec::with_capacity(items.len());
        for array in items {
            pts.push(array.into())
        }
        Pts {pts }
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

// brute force distance
fn min_dist_brute_force(ln: &Vec<Pt>, ln2: &Vec<Pt>) -> f64 {
    let mut dist = std::f64::MAX;
    let mut bln = false;
    let (n1, n2) = (ln.len() - 1, ln2.len() - 1);
    let (mut i, mut j) = (0usize, 0);
    while !bln && i < n1 {
        while !bln && j < n2 {
            let d = seg_seg_distance(ln[i], ln[i + 1], ln2[j], ln2[j + 1]);
            if d < dist {
                dist = d;
            }
            //dist = minf64(dist, d)
            bln = (dist == 0.0);
            j += 1;
        }
        i += 1;
    }
    dist
}


//Distance betwen two segments with custom hypot function
fn seg_seg_distance(sa: Pt, sb: Pt, oa: Pt, ob: Pt) -> f64 {
    let mut dist = std::f64::NAN;
    let (x1, y1) = (sa.x, sa.y);
    let (x2, y2) = (sb.x, sb.y);

    let (x3, y3) = (oa.x, oa.y);
    let (x4, y4) = (ob.x, ob.y);


    let denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    let numera = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
    let numerb = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

    let mut pta = sa;
    let mut ptb = sb;
    if (denom.abs()) < EPSILON {
        let is_aspt_a = sa.equals(&sb);
        let is_aspt_b = oa.equals(&ob);

        if is_aspt_a && is_aspt_b {
            dist = (x1 - x4).hypot(y1 - y4)
        } else if is_aspt_a || is_aspt_b {

            let (lna, lnb) = if is_aspt_a {
                pta = sa;
                (oa, ob)
            } else if is_aspt_b {
                pta = oa;
                (sa, sb)
            } else {
                unreachable!()
            };

            dist = distance_to_point(lna, lnb, pta)
        } else {
            dist = min_dist_segment_endpoints(sa, sb, oa, ob)
        }
    } else {
        let (mut use_pta, mut use_ptb) = (false, false);
        let mua = numera / denom;
        let mua = snap_to_zero_or_one(mua);

        let mub = numerb / denom;
        let mub = snap_to_zero_or_one(mub);

        if mua < 0f64 || mua > 1f64 || mub < 0f64 || mub > 1f64 {
            //the is intersection along the the segments
            if mua < 0f64 {
                pta = sa;
                use_pta = true;
            } else if mua > 1f64 {
                pta = sb;
                use_pta = true;
            }

            if mub < 0f64 {
                ptb = oa;
                use_ptb = true;
            } else if mub > 1f64 {
                ptb = ob;
                use_ptb = true;
            }

            if use_pta && use_ptb {
                dist = f64::min(
                    distance_to_point(oa, ob, pta),
                    distance_to_point(sa, sb, ptb),
                )
            } else if use_pta {
                dist = distance_to_point(oa, ob, pta)
            } else {
                dist = distance_to_point(sa, sb, ptb)
            }
        } else {
            dist = 0f64; //lines intersect
        }
    }
    dist
}

fn min_dist_segment_endpoints(sa: Pt, sb: Pt, oa: Pt, ob: Pt) -> f64 {
    let o_sa = distance_to_point(oa, ob, sa);
    let o_sb = distance_to_point(oa, ob, sb);
    let s_oa = distance_to_point(sa, sb, oa);
    let s_ob = distance_to_point(sa, sb, ob);
    (o_sa.min(o_sb)).min((s_oa.min(s_ob)))
}

//Distance from segment endpoints to point
fn distance_to_point(sa: Pt, sb: Pt, pt: Pt) -> f64 {
    let (ax, ay) = (sa.x, sa.y);
    let (bx, by) = (sb.x, sb.y);
    let (px, py) = (pt.x, pt.y);
    let (dx, dy) = (bx - ax, by - ay);
    let isz_x = dx.feq(0f64);
    let isz_y = dy.feq(0f64);

    if isz_x && isz_y {
        //line with zero length
        (px - ax).hypot(py - ay)
    } else {
        let u = (((px - ax) * dx) + ((py - ay) * dy)) / (dx * dx + dy * dy);

        let (c_ptx, c_pty) = if u < 0.0 {
            (ax, ay)
        } else if u > 1f64 {
            (bx, by)
        } else {
            (ax + u * dx, ay + u * dy)
        };
        (px - c_ptx).hypot(py - c_pty)
    }
}

#[inline]
///Snap value to zero
pub fn snap_to_zero(x: f64) -> f64 {
    if x.feq(0.0) { 0.0 } else { x }
}

#[inline]
///Snap value to zero or one
pub fn snap_to_zero_or_one(x: f64) -> f64 {
    if x.feq(0.0) { 0.0 } else if x.feq(1.0) { 1.0 } else { x }
}

