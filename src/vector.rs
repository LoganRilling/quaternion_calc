use crate::quaternion::Quaternion;

pub struct Vector
{
    pub dims: Vec<f64>
}

impl Clone for Vector
{
    fn clone(&self) -> Vector
    {
        return Vector
        {
            dims: self.dims.clone()
        }
    }
}

pub trait VectorT
{
    type Output;

    fn dot(&self, other: &Self) -> Result<f64, &'static str>;
    fn add(&self, other: &Self) -> Result<Self::Output, &'static str>;
    fn subtract(&self, other: &Self) -> Result<Self::Output, &'static str>;
    fn get_magnitude(&self) -> f64;
    fn normalize(&self) -> Result<Self::Output, &'static str>;
    fn get_angle_rad(&self, other: &Self) -> f64;
}

impl Vector
{
    pub fn new(dimensions: Vec<f64>) -> Vector
    {
        return Vector 
        {
            dims: dimensions
        };
    }

    pub fn new_from_quaternion(q: &Quaternion) -> Vector
    {
        return Self::new(vec!(q.x, q.y, q.z));
    }

    fn cross(&self, other: &Vector) -> Result<Vector, &'static str>
    {
        if self.dims.len() != 3
        {
            return Err("Attempted to find the cross product for a Vector of a dimension not equal to 3.");
        }
        
        if self.dims.len() != other.dims.len()
        {
            return Err("Attempted to find the cross product for Vectors of different dimensions");
        }

        let mut dims = vec!(0.0, 0.0, 0.0);

        dims[0] = (self.dims[1] * other.dims[2]) - (self.dims[2] * other.dims[1]);
        dims[1] = -1.0 * ((self.dims[0] * other.dims[2]) - (self.dims[2] * other.dims[0]));
        dims[2] = (self.dims[0] * other.dims[1]) - (self.dims[1] * other.dims[0]);

        return Ok(Self::new(dims));
    }
}

impl VectorT for Vector
{
    type Output = Vector;
    
    fn dot(&self, other: &Vector) -> Result<f64, &'static str>
    {
        if self.dims.len() != other.dims.len()
        {
            return Err("Attempted to find the dot product for Vectors of different dimensions");
        }

        let mut sum = 0f64;

        for i in 0..self.dims.len()
        {
            sum += self.dims.get(i).expect("Index outside range!") * other.dims.get(i).expect("Index outside range!");
        };

        return Ok(sum);
    }

    fn add(&self, other: &Vector) -> Result<Vector, &'static str>
    {
        if self.dims.len() != other.dims.len()
        {
            return Err("Attempted to add Vectors of different dimensions");
        }

        let mut dims = Vec::with_capacity(self.dims.len());
        
        for i in 0..(self.dims.len())
        {
            dims.push(self.dims[i] + other.dims[i]);
        }

        return Ok(Self::new(dims));
    }

    fn subtract(&self, other: &Vector) -> Result<Vector, &'static str>
    {
        if self.dims.len() != other.dims.len()
        {
            return Err("Attempted to subtract Vectors of different dimensions");
        }

        let mut dims = Vec::with_capacity(self.dims.len());
        
        for i in 0..(self.dims.len())
        {
            dims.push(self.dims[i] - other.dims[i]);
        }

        return Ok(Self::new(dims));
    }

    fn get_magnitude(&self) -> f64
    {
        return Self::dot(&self, self).unwrap().sqrt();
    }

    fn normalize(&self) -> Result<Vector, &'static str>
    {   
        let mag = self.get_magnitude();

        if mag == 0.0
        {
            return Err("Can not normalize a vector of magnitude 0.");
        }

        let mut dims = Vec::with_capacity(self.dims.len());
        for i in 0..(self.dims.len())
        {
            dims.push(self.dims[i]/mag);
        }
        

        return Ok(Self::new(dims));
    }

    fn get_angle_rad(&self, other: &Vector) -> f64
    {
        return (Self::dot(self, other).expect("Failed to get the dot product for getting the angle compared to another vector.") /
        (self.get_magnitude() * other.get_magnitude())).acos();
    }
}