//!
//! Zone Drawing Example - Rust (Renderer)
//! ========================================
//! Renders all 7 ball configuration zones from the curriculum training table
//! in a **single image** using the MuJoCo-rs offscreen renderer with the
//! ``cam_top_closeup`` camera. Each zone gets a distinct color and a numeric
//! label (1–7) rendered at its center.
//!
//! Output: zones.png
//!
//! Before running this example, some dependencies must be set.
//!
//! First, move to the root of the repository (not the example).
//! Then fetch the MuJoCo dependency:
//! - Linux: ./fetch_mujoco_linux.sh
//! - Windows: .\fetch_mujoco_windows.ps1
//! - MacOS: ./fetch_mujoco_macos.sh
//!
//! Then export variables (needs to be re-run every time the terminal is closed):
//! - Linux: source setup_linux.sh
//! - Windows: .\setup_windows.ps1
//! - MacOS: source setup_macos.sh
//!
//! You should now be able to run the example.
//!
use fuzbai_simulator::mujoco_rs::{
    prelude::{MjData, MjModel, MjtFontScale, MjtObj, MjvCamera},
    renderer::MjRenderer,
};
use fuzbai_simulator::render::Visualizer;
use std::io::BufWriter;

/// Ball configuration zones from the curriculum training table (local mm coords).
/// Each entry is (x_min, x_max, y_min, y_max).
const BALL_CONFIG_ZONES: [(f64, f64, f64, f64); 7] = [
    (820.0, 860.0, 100.0, 600.0),
    (880.0, 940.0, 100.0, 600.0),
    (720.0, 780.0, 100.0, 600.0),
    (950.0, 1100.0, 100.0, 600.0),
    (560.0, 710.0, 100.0, 600.0),
    (1000.0, 1100.0, 100.0, 600.0),
    (560.0, 600.0, 100.0, 600.0),
];

/// Distinct, perceptually separated RGBA colors (semi-transparent) for each zone.
/// Overlapping pairs use maximally contrasting hues:
///  - Zone 6 (1000–1100) sits inside zone 4 (950–1100): yellow inside deep-blue
///  - Zone 7 (560–600)   sits inside zone 5 (560–710):  cyan inside orange
const ZONE_COLORS: [[f32; 4]; 7] = [
    [1.00, 1.00, 0.00, 0.75], // 1 - bright yellow
    [1.00, 0.00, 0.80, 0.75], // 2 - hot magenta
    [0.10, 0.90, 0.10, 0.75], // 3 - lime green
    [0.00, 0.20, 1.00, 0.75], // 4 - deep blue  (zone 6 is nested inside)
    [1.00, 0.45, 0.00, 0.75], // 5 - orange     (zone 7 is nested inside)
    [0.95, 0.95, 0.00, 0.90], // 6 - vivid yellow (high contrast on blue of zone 4)
    [0.00, 0.95, 1.00, 0.90], // 7 - cyan        (high contrast on orange of zone 5)
];

/// Z-offsets (metres above Z_FIELD) for each zone.
/// Zones 6 and 7 are nested inside zones 4 and 5 respectively.
/// The angled camera reveals these as visually floating above the outer zones.
const ZONE_Z_OFFSETS: [f64; 7] = [
    0.000, // 1 - standalone
    0.000, // 2 - standalone
    0.000, // 3 - standalone
    0.000, // 4 - outer (zone 6 nested inside)
    0.000, // 5 - outer (zone 7 nested inside)
    0.010, // 6 - nested inside zone 4
    0.010, // 7 - nested inside zone 5
];

fn main() {
    // Determine the path to the model XML.
    // Resolved relative to the workspace root (where cargo is run from).
    let model_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../models/miza.xml");

    // Load the model and create physics state.
    let model = MjModel::from_xml(model_path).expect("failed to load miza.xml");
    let mut data = MjData::new(&model);
    // Run forward kinematics so all positions are up to date.
    data.step();

    // Use the angled camera so perspective depth separates nested zones.
    let cam_id = model.name_to_id(MjtObj::mjOBJ_CAMERA, "cam_zone_angled") as u32;
    let camera = MjvCamera::new_fixed(cam_id);

    // Build the offscreen renderer. 5 geom slots per zone (1 ghost fill + 4 border bars).
    let mut renderer = MjRenderer::builder()
        .camera(camera)
        .num_visual_user_geom(BALL_CONFIG_ZONES.len() as u32 * 5)
        .build(&model)
        .expect("failed to create renderer");

    // Set a large font scale so labels are clearly legible.
    renderer.set_font_scale(MjtFontScale::mjFONTSCALE_200);

    // Draw all zones into the user scene using border-only outlines.
    // Zone 4's label is shifted left so it doesn't overlap with zone 6.
    const ZONE_LABEL_X_OFFSET: [f64; 7] = [0.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0];
    for (i, (((&zone, &color), &z_off), &lx)) in BALL_CONFIG_ZONES
        .iter()
        .zip(ZONE_COLORS.iter())
        .zip(ZONE_Z_OFFSETS.iter())
        .zip(ZONE_LABEL_X_OFFSET.iter())
        .enumerate()
    {
        let label = format!("{}", i + 1);
        Visualizer::<&MjModel>::render_zone_outline(
            renderer.user_scene_mut(),
            zone,
            Some(color),
            Some(&label),
            6.0,   // border thickness in local mm
            z_off, // z-offset (inner zones float above outer)
            lx,    // label x offset in local mm
        );
    }

    // Sync scene with the physics state and render offscreen.
    renderer.sync(&mut data);

    // MuJoCo renders text labels as 2D screen-space overlays (OpenGL y-up convention).
    // The renderer's internal flip_image_vertically fixes the 3D scene orientation but
    // inverts the text. We undo this by flipping the pixel buffer vertically ourselves
    // before writing the PNG.
    let width = model.vis().global.offwidth as usize;
    let height = model.vis().global.offheight as usize;
    let src = renderer.rgb_flat().expect("RGB rendering not enabled");
    let mut flipped = vec![0u8; src.len()];
    for row in 0..height {
        let src_row = &src[row * width * 3..(row + 1) * width * 3];
        let dst_row = &mut flipped[(height - 1 - row) * width * 3..(height - row) * width * 3];
        dst_row.copy_from_slice(src_row);
    }

    let file = std::fs::File::create("zones.png").expect("failed to create zones.png");
    let mut enc = png::Encoder::new(BufWriter::new(file), width as u32, height as u32);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header()
        .expect("PNG header")
        .write_image_data(&flipped)
        .expect("PNG data");
    println!("Saved zones.png");
}
