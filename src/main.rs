mod quaternion;
use quaternion::Quaternion;

use std::f64::consts::PI;


fn rad_to_deg(degrees: f64) -> f64
{
    return 180.0 * degrees / PI;
}

fn deg_to_rad(radians: f64) -> f64
{
    return PI * radians / 180.0;
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

fn display_quaternion_deg(q: Quaternion)
{
    let w = rad_to_deg(q.w);
    let x = rad_to_deg(q.x);
    let y = rad_to_deg(q.y);
    let z = rad_to_deg(q.z);

    println!("w: {w}");
    println!("x: {x}");
    println!("y: {y}");
    println!("z: {z}");
}

fn main() 
{
    let q0 = Quaternion::new_from_pitch(deg_to_rad(90.0));
    let q1 = Quaternion::new_from_pitch(deg_to_rad(180.0));
    let p = q0.get_rotation_to(&q1);

    let a0 = rad_to_deg(q0.get_angle());
    let a1 = rad_to_deg(q1.get_angle());
    //let ap = rad_to_deg(p.get_angle());
    let o = rad_to_deg((q0 * p).get_angle());

    println!("q_0 pitch: {a0}");
    println!("q_1 pitch: {a1}");
    println!("q0 * p = q1 pitch: {o}");
}
