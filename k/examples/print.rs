#[macro_use]
extern crate k;
extern crate nalgebra as na;

use na::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use k::prelude::*;
use k::{JacobianIKSolver, JointBuilder, JointType};


fn main() {
    let fixed: k::Node<f32> = JointBuilder::new()
        .name("fixed")
        .joint_type(JointType::Fixed)
        .translation(Translation3::new(0.0, 0.0, 0.1))
        .finalize()
        .into();
    let l0: k::Node<f32> = JointBuilder::new()
        .name("torso_linear")
        .joint_type(JointType::Linear {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.1, 0.0, 0.1))
        .finalize()
        .into();
    let l1: k::Node<f32> = JointBuilder::new()
        .name("shoulder_yaw")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.3, 0.0, 0.0))
        .finalize()
        .into();
    let l2: k::Node<f32> = JointBuilder::new()
        .name("elbow_yaw")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.3, 0.0, 0.0))
        .finalize()
        .into();
    let l3: k::Node<f32> = JointBuilder::new()
        .name("wrist_yaw")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.3, 0.0, 0.0))
        .finalize()
        .into();

    connect![fixed => l0 => l1 => l2 => l3];

    let tree = k::Chain::from_root(fixed);
    println!("{}", tree);
}
