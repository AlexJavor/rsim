use nphysics2d::algebra::{Force2, ForceType};
use nphysics2d::object::RigidBody;

pub struct Robot {
    acc_lim_x: f64,
    acc_lim_y: f64,
    acc_lim_theta: f64,
    max_vel_x: f64,
    min_vel_x: f64,
    max_vel_theta: f64,
    min_vel_theta: f64,
    min_in_place_vel_theta: f64,
    escape_vel: f64,
    holonomic_robot: bool,
    y_vels: [f64; 4],
}

impl Default for Robot {
    fn default() -> Self {
        Robot {
            acc_lim_x: 2.5,
            acc_lim_y: 2.5,
            acc_lim_theta: 3.2,
            max_vel_x: 0.5,
            min_vel_x: 0.1,
            max_vel_theta: 1.0,
            min_vel_theta: -1.0,
            min_in_place_vel_theta: 0.4,
            escape_vel: 0.1,
            holonomic_robot: false,
            y_vels: [-0.3, -0.1, 0.1, 0.3]
        }
    }
}

type R = f64;

pub trait Position2D<R> {
    fn x(&self) -> R;
    fn y(&self) -> R;
    fn theta(&self) -> R;
}

impl<R: na::RealField> Position2D<R> for RigidBody<R> {
    fn x(&self) -> R {
        self.position().translation.x
    }

    fn y(&self) -> R {
        self.position().translation.y
    }

    fn theta(&self) -> R {
        self.position().rotation.angle()
    }
}

pub trait Veloc<R> {
    fn forward_vel(&self) -> R;
    fn lateral_vel(&self) -> R;
    fn rotational_vel(&self) -> R;
}

impl <R: na::RealField> Veloc<R> for RigidBody<R> {
    fn forward_vel(&self) -> R {
        let theta = self.theta();
        let dot_x = self.velocity().linear[0];
        let dot_y = self.velocity().linear[1];
        let v_x = dot_x * theta.cos();
        let v_y = dot_y * theta.sin();
        v_x + v_y
    }

    fn lateral_vel(&self) -> R {
        let dot_x = self.velocity().linear[0];
        let dot_y = self.velocity().linear[1];
        let v_x = dot_x * self.theta().sin();
        let v_y = - dot_y * self.theta().cos();
        v_x + v_y
    }

    fn rotational_vel(&self) -> R {
        self.velocity().angular
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Command {
    pub forward_velocity: R,
    pub steering_angle: R
}

