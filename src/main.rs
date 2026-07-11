mod vector;
mod quaternion;

use vector::Vector;
use quaternion::Quaternion;

use std::f64::consts::PI;

fn display_vector(v: Vector)
{
    for i in 0..(v.dims.len())
    {
        let val = v.dims[i];
        println!("Dimension {i}: {val}");
    }
}

fn display_quaternion(q: Quaternion)
{
    let w = q.w;
    let x = q.x;
    let y = q.y;
    let z = q.z;

    println!("w: {w}");
    println!("x: {x}");
    println!("y: {y}");
    println!("z: {z}");
}

fn main() 
{
    
}
