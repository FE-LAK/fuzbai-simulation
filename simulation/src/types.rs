/// Type definitions
/// Represents the return type of [`FuzbAISimulator::observation`].
pub type ObservationType = (f64, f64, f64, f64, [f64;8], [f64;8]);
/// Represents a 3D Cartesian position
pub type XYZType = [f64; 3];
/// Represents a 2D Cartesian position
pub type XYType = [f64; 2];
/// Represents a single motor/rod command.
/// `(motor index (starts at 1), target position, target rotation, trans velocity (ignored), rot velocity (ignored))`
pub type MotorCommand = (usize, f64, f64, f64, f64);
/// Represents a placeholder for RGBA values.
pub type RGBAType = [f32; 4];
/// Represents a tuple containing positions of the ball and the rods.
pub type TraceType = ([f64; 3], [f64;8], [f64;8]);
