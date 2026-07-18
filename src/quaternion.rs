use std::ops::{Mul, Add};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion
{
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    unit_quaternion: bool
}

#[allow(dead_code)]
impl Quaternion
{
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Quaternion
    {
        return Quaternion 
        {
            w,
            x,
            y,
            z,
            unit_quaternion: false
        };
    }

    
    /// Creates a quaternion and enables the unit quaternion property on by default.
    /// 
    /// A unit quaternion ensures that the magnitude of the quaternion is always 1, 
    /// no matter the operations conducted onto it. It simply saves computational headaches.
    pub fn new_unit_quat(w: f64, x: f64, y: f64, z: f64) -> Quaternion
    {
        return Quaternion 
        {
            w,
            x,
            y,
            z,
            unit_quaternion: true
        }.normalize().expect("Normalized quaternion");
    }

    /// Sets the unit quaternion property on/off
    pub fn set_unit_quat(mut self, state: bool)
    {
        self.unit_quaternion = state;

        if self.unit_quaternion
        {
            self = self.normalize().expect("Normalized quaternion");
        }
    }

    /// Unit quaternion that represents a rotation over x-axis
    pub fn new_from_roll(angle_rad: f64)  -> Quaternion
    {
        return Self::new_unit_quat((angle_rad * 0.5).cos(), (angle_rad * 0.5).sin(), 0.0, 0.0);
    }

    /// Unit quaternion that represents a rotation over y-axis
    pub fn new_from_pitch(angle_rad: f64)  -> Quaternion 
    {
        return Self::new_unit_quat((angle_rad * 0.5).cos(),0.0, (angle_rad * 0.5).sin(), 0.0);
    }

    /// Unit quaternion that represents a rotation over z-axis
    pub fn new_from_yaw(angle_rad: f64)  -> Quaternion
    {
        return Self::new_unit_quat((angle_rad * 0.5).cos(), 0.0, 0.0, (angle_rad * 0.5).sin());
    }

    /// Creates a unit quaternion from Euler angles using the
    /// ZYX (yaw → pitch → roll) convention.
    pub fn new_from_euler(yaw_z_rad: f64, pitch_y_rad: f64, roll_x_rad: f64) -> Quaternion
    {
        return Self::new_from_yaw(yaw_z_rad) * 
               Self::new_from_pitch(pitch_y_rad) *
               Self::new_from_roll(roll_x_rad);
    }

    pub fn to_euler(&self) -> (f64, f64, f64)
    {
        let unit = self.normalize().expect("Normalization in to_euler");

        return (
            (2.0 * (unit.w * unit.x + unit.y * unit.z))
                .atan2(1.0 - 2.0 * (unit.x * unit.x + unit.y * unit.y)),

            (2.0 * (unit.w * unit.y - unit.z * unit.x))
                .clamp(-1.0, 1.0).asin(),
            
            (2.0 * (unit.w * unit.z + unit.x * unit.y))
                .atan2(1.0 - 2.0 * (unit.y * unit.y + unit.z * unit.z))
        );
    }

    pub fn get_conjugate(&self) -> Quaternion 
    {
        return Quaternion::new(
             self.w,
            -self.x,
            -self.y,
            -self.z,
        )
    }

    /// Creates a transitional or error quaternion. 
    /// Gets the differential quaternion components to be used for a rotation
    pub fn get_rotation_to(&self, target: &Quaternion) -> Quaternion
    {
        let q0 = self.clone();
        let q1 = target.clone();

        if !q0.unit_quaternion
        {
            q0.set_unit_quat(true);
        }

        if !q1.unit_quaternion
        {
            q1.set_unit_quat(true);
        }

        return q1.hamilton_product(q0.get_conjugate());
    }

    pub fn hamilton_product(&self, other: Quaternion) -> Quaternion
    {
        let q = Self::new(
            (self.w * other.w) -
                (self.x * other.x) -
                (self.y * other.y) -
                (self.z * other.z),
            (self.w * other.x) + (other.w * self.x) + (self.y * other.z) - (self.z * other.y),
            (self.w * other.y) + (other.w * self.y) - ((self.x * other.z) - (self.z * other.x)),
            (self.w * other.z) + (other.w * self.z)+ (self.x * other.y) - (self.y * other.x),
        );

        if self.unit_quaternion
        {
            return q.normalize().expect("Return normalized hamilton_product");
        }
        else 
        {
            return q;    
        }
    }

    pub fn dot(&self, other: &Quaternion) -> f64
    {
        return (self.w * other.w) + (self.x * other.x) + (self.y * other.y) + (self.z * other.z);
    }

    pub fn add(&self, other: &Quaternion) -> Quaternion
    {
        return Self::new(self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z);
    }

    pub fn subtract(&self, other: &Quaternion) -> Quaternion
    {
        return Self::new(self.w - other.w, self.x - other.x, self.y - other.y, self.z - other.z);
    }

    pub fn get_magnitude(&self) -> f64
    {
        return Self::dot(&self, self).sqrt();
    }

    pub fn normalize(&self) -> Result<Quaternion, &'static str>
    {
        let mag = self.get_magnitude();

        if mag == 0.0
        {
            return Err("Can not normalize a quaternion of magnitude 0.");
        }

        return Ok(Self::new(self.w/mag, self.x/mag, self.y/mag, self.z/mag));
    }

    /// Gets the current angle in radians
    pub fn get_angle(&self) -> f64
    {
        return 2.0 * self.w.acos();
    }
}

impl Mul for Quaternion
{
    type Output = Quaternion;

    fn mul(self, rhs: Self) -> Self::Output 
    {
        return self.hamilton_product(rhs);
    }
}

#[allow(unconditional_recursion)]
impl Add for Quaternion
{
    type Output = Quaternion;

    fn add(self, rhs: Self) -> Self::Output 
    {
        return self.add(rhs);
    }
}

/// Scalar multiplicaton for quaternions
impl Mul<Quaternion> for f64
{
    type Output = Quaternion;

    fn mul(self, rhs: Quaternion) -> Quaternion
    {
        Quaternion::new(
            rhs.w * self,
            rhs.x * self,
            rhs.y * self,
            rhs.z * self,
        )
    }
}