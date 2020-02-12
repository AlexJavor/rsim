use nphysics2d::algebra::Force2;
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

pub trait Pos<R> {
    fn x(&self) -> R;
    fn y(&self) -> R;
    fn theta(&self) -> R;
}

impl<R: na::RealField> Pos<R> for RigidBody<R> {
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

pub trait Vel<R> {
    fn forward(&self) -> R;
    fn lateral(&self) -> R;
    fn rotational(&self) -> R;
}

impl <R: na::RealField> Vel<R> for RigidBody<R> {
    fn forward(&self) -> R {
        let theta = self.theta();
        let dot_x = self.velocity().linear[0];
        let dot_y = self.velocity().linear[1];
        let v_x = dot_x * theta.cos();
        let v_y = dot_y * theta.sin();
        v_x + v_y
    }

    fn lateral(&self) -> R {
        let dot_x = self.velocity().linear[0];
        let dot_y = self.velocity().linear[1];
        let v_x = dot_x * self.theta().sin();
        let v_y = - dot_y * self.theta().cos();
        v_x + v_y
    }

    fn rotational(&self) -> R {
        self.velocity().angular
    }
}

pub struct Command<R> {
    pub forward_velocity: R,
    pub steering_angle: R
}

pub fn compute_acceleration<T: Pos<R> + Vel<R>>(cmd: &Command<R>, pos_vel: &T, robot: &Robot) -> Result<Force2<R>, u32> {
    let target_vel = cmd.forward_velocity.min(robot.max_vel_x);
    let diff = target_vel - pos_vel.forward();
    let dt = 1. / 60.;

    let accel1 = diff / dt;
    let accel = accel1.min(robot.acc_lim_x).max(-robot.acc_lim_x);

    let accel_x = accel * pos_vel.theta().cos();
    let accel_y = accel * pos_vel.theta().sin();
    let accel_theta = pos_vel.forward() * cmd.steering_angle.tan() * 1.;

    let a = Force2 {
        linear: na::Vector2::new(accel_x, accel_y),
        angular: accel_theta
    };

    Ok(a)
}

