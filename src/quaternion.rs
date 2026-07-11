use crate::vector::{Vector, VectorT};
use std::ops::Mul;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion
{
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64
}

impl Quaternion
{
    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Quaternion
    {
        return Quaternion 
        {
            w,
            x,
            y,
            z
        };
    }

    pub fn new_from_vector(v: &Vector) -> Result<Quaternion, &'static str>
    {
        if v.dims.len() != 3
        {
            return Err("Can only create a quaternion from a vector of exactly 3 dimensions!");
        }

        return Ok(Quaternion 
        {
            w: 0.0,
            x: v.dims[0],
            y: v.dims[1],
            z: v.dims[2]
        });
    }

    /// Rotation over x-axis
    pub fn new_from_roll(angle_rad: f64)  -> Quaternion
    {
        return Self::new((angle_rad * 0.5).cos(), (angle_rad * 0.5).sin(), 0.0, 0.0);
    }

    /// Rotation over y-axis
    pub fn new_from_pitch(angle_rad: f64)  -> Quaternion 
    {
        return Self::new((angle_rad * 0.5).cos(),0.0, (angle_rad * 0.5).sin(), 0.0);
    }

    /// Rotation over z-axis
    pub fn new_from_yaw(angle_rad: f64)  -> Quaternion
    {
        return Self::new((angle_rad * 0.5).cos(), 0.0, 0.0, (angle_rad * 0.5).sin());
    }

    /// Creates a quaternion from Euler angles using the
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

    pub fn get_inverse(&self) -> Result<Quaternion, &'static str> 
    {
        let conj = self.get_conjugate();
        let mag = self.get_magnitude();

        if mag == 0.0
        {
            return Err("Can not divide by zero!");
        }

        return Ok(Quaternion::new(
            conj.w / (mag * mag),
            conj.x / (mag * mag),
            conj.y / (mag * mag),
            conj.z / (mag * mag),
        ));
    }

    pub fn get_rotation_error(&self, target: &Quaternion) -> Quaternion
    {
        return target.normalize().unwrap() * self.normalize().unwrap().get_conjugate();
    }

    pub fn rotate(&mut self, target: &Quaternion)
    {
        let mut mag = self.get_magnitude();

        *self = mag * (target.normalize().unwrap() * self.normalize().unwrap());
    }

    pub fn to_euler_as_vector(&self) -> Vector
    {
        let (x, y, z) = self.to_euler();

        return Vector::new(vec!(x, y, z));
    }

    pub fn hamilton_product(&self, other: Quaternion) -> Quaternion
    {
        return Self::new(
            (self.w * other.w) -
                (self.x * other.x) -
                (self.y * other.y) -
                (self.z * other.z),
            (self.w * other.x) + (other.w * self.x) + (self.y * other.z) - (self.z * other.y),
            (self.w * other.y) + (other.w * self.y) - ((self.x * other.z) - (self.z * other.x)),
            (self.w * other.z) + (other.w * self.z)+ (self.x * other.y) - (self.y * other.x),
        );
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

impl VectorT for Quaternion
{
    type Output = Quaternion;

    fn dot(&self, other: &Quaternion) -> Result<f64, &'static str>
    {
        return Ok((self.w * other.w) + (self.x * other.x) + (self.y * other.y) + (self.z * other.z));
    }

    fn add(&self, other: &Quaternion) -> Result<Quaternion, &'static str>
    {
        return Ok(Self::new(self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z));
    }

    fn subtract(&self, other: &Quaternion) -> Result<Quaternion, &'static str>
    {
        return Ok(Self::new(self.w - other.w, self.x - other.x, self.y - other.y, self.z - other.z));
    }

    fn get_magnitude(&self) -> f64
    {
        return Self::dot(&self, self).unwrap().sqrt();
    }

    fn normalize(&self) -> Result<Quaternion, &'static str>
    {
        let mag = self.get_magnitude();

        if mag == 0.0
        {
            return Err("Can not normalize a quaternion of magnitude 0.");
        }

        return Ok(Self::new(self.w/mag, self.x/mag, self.y/mag, self.z/mag));
    }

    fn get_angle_rad(&self, other: &Quaternion) -> f64
    {
        return (Self::dot(self, other).unwrap() /
        (self.get_magnitude() * other.get_magnitude())).acos();
    }
}