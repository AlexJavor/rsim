use ggez::nalgebra::Isometry2;
use nphysics2d::algebra::Velocity2;

#[derive(Copy, Clone, Default, Debug)]
pub struct Point {
    pub x: f32,
    pub y: f32
}

impl Into<[f32; 2]> for Point {
    fn into(self) -> [f32; 2] {
        [self.x as f32, self.y as f32]
    }
}

#[derive(Clone, Default, Debug)]
#[repr(C)]
pub struct PointCloud {
    pub points: Vec<Point>
}

#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32
}

impl From<Isometry2<f64>> for Pose {
    fn from(i: Isometry2<f64>) -> Self {
        Pose {
            x: i.translation.vector[0] as f32,
            y: i.translation.vector[1] as f32,
            theta: i.rotation.angle() as f32
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Vel {
    pub x: f64,
    pub y: f64,
    pub theta: f64
}


impl From<Velocity2<f64>> for Vel {
    fn from(v: Velocity2<f64>) -> Self {
        Vel {
            x: v.linear[0],
            y: v.linear[1],
            theta: v.angular
        }
    }
}
