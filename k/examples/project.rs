#[macro_use]
extern crate k;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate indicatif;

extern crate csv;
// #[macro_use]
extern crate serde;

use serde::{Serialize, Deserialize};
use csv::WriterBuilder;
use std::fs::File;
use std::io::Write;
use std::io;

use indicatif::{ProgressBar, ProgressStyle};
// use kiss3d::event::{Action, Key, WindowEvent};
use k::prelude::*;
use k::{JacobianIKSolver, JointBuilder, JointType};
// use kiss3d::camera::ArcBall;
// use kiss3d::light::Light;
// use kiss3d::scene::SceneNode;
// use kiss3d::window::Window;
use na::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

#[derive(Serialize, Deserialize, Debug)]
struct Record {
    l0: Vec<f32>,
    l1: Vec<f32>,
    l2: Vec<f32>,
    l3: Vec<f32>,
    l4: Vec<f32>,
    l5: Vec<f32>,
    l6: Vec<f32>,
    l0_final: Vec<f32>,
    l1_final: Vec<f32>,
    l2_final: Vec<f32>,
    l3_final: Vec<f32>,
    l4_final: Vec<f32>,
    l5_final: Vec<f32>,
    l6_final: Vec<f32>,
}

fn main() {
    let fixed: k::Node<f32> = JointBuilder::new()
        .name("fixed")
        .joint_type(JointType::Fixed)
        .translation(Translation3::new(0.0, 0.0, 0.6))
        .finalize()
        .into();
    let l0: k::Node<f32> = JointBuilder::new()
        .name("shoulder_pitch")
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize()
        .into();
    let l1: k::Node<f32> = JointBuilder::new()
        .name("shoulder_roll")
        .joint_type(JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize()
        .into();
    let l2: k::Node<f32> = JointBuilder::new()
        .name("shoulder_yaw")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.30))
        .finalize()
        .into();
    let l3: k::Node<f32> = JointBuilder::new()
        .name("elbow_pitch")
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l4: k::Node<f32> = JointBuilder::new()
        .name("wrist_yaw")
        .joint_type(JointType::Rotational {
            axis: Vector3::z_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l5: k::Node<f32> = JointBuilder::new()
        .name("wrist_pitch")
        .joint_type(JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.15))
        .finalize()
        .into();
    let l6: k::Node<f32> = JointBuilder::new()
        .name("wrist_roll")
        .joint_type(JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.0, 0.0, -0.10))
        .finalize()
        .into();

    connect![fixed => l0 => l1 => l2 => l3 => l4 => l5 => l6];

    let arm = k::SerialChain::new_unchecked(
        k::Chain::from_root(fixed)
    );

    let angles = vec![0.2, 0.2, 0.0, -1.5, 0.0, -0.3, 0.0];

    arm.set_joint_positions(&angles).expect("Couldn't set joint positions.");
    let base_rot = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, -0.6), 
        UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57)
    );

    arm.iter().next().unwrap().set_origin(
        base_rot
            * Isometry3::from_parts(
                Translation3::new(0.0, 0.0, 0.6),
                UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
            ),
    );
    arm.update_transforms();

    let end = arm.find("wrist_roll").expect("Can't find wrist_roll for end.");
    let mut target = end.world_transform()
        .expect("Can't find world_transform for target").clone();

    let solver = JacobianIKSolver::default();
    let mut constraints = k::Constraints::default();
    constraints.rotation_x = false;
    
    let mut wtr = WriterBuilder::new().has_headers(false).from_writer(vec![]);

    let pb = ProgressBar::new(200);
    pb.set_style(ProgressStyle::default_bar()
        .template("{spinner:.green} [{elapsed_precise}] [{bar:100.cyan/blue}] {bytes}/{total_bytes} ({eta})")
        .progress_chars("=>-"));

    for x in -100..100 {
        for y in -100..100 {
            for z in -100..100 {
                target.translation.vector[0] += (x as f32) * 0.1;
                target.translation.vector[1] += (y as f32) * 0.1;
                target.translation.vector[2] += (z as f32) * 0.1;
                // println!("Target is {:?}", target.translation.vector.data);
                let _l0 = l0.world_transform().expect("_l0").translation.vector.data.to_vec(); 
                let _l2 = l2.world_transform().expect("_l1").translation.vector.data.to_vec();
                let _l1 = l1.world_transform().expect("_l2").translation.vector.data.to_vec();
                let _l4 = l4.world_transform().expect("_l3").translation.vector.data.to_vec();
                let _l3 = l3.world_transform().expect("_l4").translation.vector.data.to_vec();
                let _l6 = l6.world_transform().expect("_l5").translation.vector.data.to_vec();
                let _l5 = l5.world_transform().expect("_l6").translation.vector.data.to_vec();

                match solver.solve_with_constraints(&arm, &target, &constraints) {
                    Ok(_) => {
                            arm.update_transforms();

                            wtr.serialize(Record {
                                l0: _l0, 
                                l1: _l1,
                                l2: _l2,
                                l3: _l3,
                                l4: _l4,
                                l5: _l5,
                                l6: _l6,
                                l0_final: l0.world_transform().expect("_l0_final").translation.vector.data.to_vec(),
                                l1_final: l2.world_transform().expect("_l1_final").translation.vector.data.to_vec(),
                                l2_final: l1.world_transform().expect("_l2_final").translation.vector.data.to_vec(), 
                                l3_final: l4.world_transform().expect("_l3_final").translation.vector.data.to_vec(),
                                l4_final: l3.world_transform().expect("_l4_final").translation.vector.data.to_vec(), 
                                l5_final: l6.world_transform().expect("_l5_final").translation.vector.data.to_vec(),
                                l6_final: l5.world_transform().expect("_l6_final").translation.vector.data.to_vec(),
                            }).expect("Couldn't serialize");
                    },
                    Err(_) => {
                        // println!("Err");
                    }
                }

                arm.set_joint_positions(&angles).unwrap();
                arm.update_transforms();
                target = end.world_transform().unwrap().clone();
            }
            // pb.tick();
        }
        pb.inc(1);
    }

    let data = String::from_utf8(wtr.into_inner().unwrap()).unwrap();
    // println!("{:?}", data);
    let mut f = File::create("data.csv").expect("Unable to create file");
    f.write_all(data.as_bytes()).expect("Unable to write data");
}