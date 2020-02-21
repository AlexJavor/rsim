use ggez::graphics::{Mesh, DrawMode, Color};
use ggez::{graphics, Context, GameResult};

pub(crate) struct Meshes {
    pub impact: Mesh,
    pub robot: Mesh,
    pub target: Mesh
}


impl Meshes {
    pub fn new(ctx: &mut Context) -> GameResult<Meshes> {
        let mesh = &mut graphics::MeshBuilder::new();
        mesh.circle(
            DrawMode::fill(),
            [0f32, 0f32],
            10.,
            0.1,
            Color::new(1.0, 1.0, 1.0, 1.0),
        );
        mesh.line(
        &[[-2.5f32, 5f32],  [7f32, 0f32], [-2.5f32, -5f32]],
        3.0,
        Color::new(1.0, 0.0, 0.0, 1.0),
        )?;
        let robot = mesh.build(ctx)?;

        let mesh = &mut graphics::MeshBuilder::new();
        mesh.circle(
           DrawMode::fill(),
            [0f32, 0f32],
            3.2,
            0.1,
            Color::new(1., 0., 1., 1.0),
        );
        let impact = mesh.build(ctx)?;
        let x = 4f32;
        let mesh = &mut graphics::MeshBuilder::new();
        mesh.line(
        &[[-x, -x],  [x, x]],
        1.0,
        Color::new(1.0, 0.0, 0.0, 1.0),
        )?;
        mesh.line(
            &[[-x, x],  [x, -x]],
            2.0,
            Color::new(1.0, 0.0, 0.0, 1.0),
        )?;
        let target = mesh.build(ctx)?;

        Ok(Meshes {
            impact,
            robot,
            target
        })
    }
}