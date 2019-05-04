#[macro_use]
extern crate k;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate indicatif;

extern crate rand;
extern crate rayon;

use std::fs::{read_to_string, remove_file};
use std::fs::File;
use std::io::Write;
use rayon::prelude::*;

use rand::{thread_rng, Rng};
use k::prelude::*;
use k::{JacobianIKSolver, JointBuilder, JointType};
use na::{Isometry3, Translation3, UnitQuaternion, Vector3};

macro_rules! init {
    (
        $fixed: ident, $l0: ident, $l1: ident, $l2: ident, $l3: ident,
        $arm: ident, $solver: ident, $constraints: ident,
        $target: ident, $angles: ident, $end: ident
    ) => {
            let $fixed: k::Node<f32> = JointBuilder::new()
                .name("fixed")
                .joint_type(JointType::Fixed)
                .translation(Translation3::new(0.0, 0.0, 0.1))
                .finalize()
                .into();
            let $l0: k::Node<f32> = JointBuilder::new()
                .name("torso_linear")
                .joint_type(JointType::Linear {
                    axis: Vector3::z_axis(),
                })
                .translation(Translation3::new(0.1, 0.0, 0.1))
                .finalize()
                .into();
            let $l1: k::Node<f32> = JointBuilder::new()
                .name("shoulder_yaw")
                .joint_type(JointType::Rotational {
                    axis: Vector3::z_axis(),
                })
                .translation(Translation3::new(0.3, 0.0, 0.0))
                .finalize()
                .into();
            let $l2: k::Node<f32> = JointBuilder::new()
                .name("elbow_yaw")
                .joint_type(JointType::Rotational {
                    axis: Vector3::z_axis(),
                })
                .translation(Translation3::new(0.3, 0.0, 0.0))
                .finalize()
                .into();
            let $l3: k::Node<f32> = JointBuilder::new()
                .name("wrist_yaw")
                .joint_type(JointType::Rotational {
                    axis: Vector3::z_axis(),
                })
                .translation(Translation3::new(0.3, 0.0, 0.0))
                .finalize()
                .into();
            connect![$fixed => $l0 => $l1 => $l2 => $l3];

            let $arm = k::SerialChain::new_unchecked(
                k::Chain::from_root($fixed)
            );

            let $angles = vec![0.2, 0.6, -1.2, 0.6];

            $arm.set_joint_positions(&$angles).expect("Couldn't set joint positions.");
            let base_rot = Isometry3::from_parts(
                Translation3::new(0.0, 0.0, 0.0),
                UnitQuaternion::from_euler_angles(0.0, -1.57, -1.57),
            );

            $arm.iter().next().unwrap().set_origin(
                base_rot
                    * Isometry3::from_parts(
                        Translation3::new(0.0, 0.0, 0.2),
                        UnitQuaternion::from_euler_angles(0.0, 0.0, 0.0),
                    ),
            );

            $arm.update_transforms();

            let $end = $arm.find("wrist_yaw").expect("Can't find wrist_yaw for end.");
            let mut $target = $end.world_transform()
                .expect("Can't find world_transform for target").clone();

            let mut $solver = JacobianIKSolver::default();
            $solver.num_max_try = 12;
            let mut $constraints = k::Constraints::default();
            $constraints.rotation_x = false;
            $constraints.rotation_z = false;   
        };
}

macro_rules! translation_of {
    ($thing: expr) => {
        {
            let v = $thing.translation.vector.data.to_vec();
            (v[0], v[1], v[2])
        }
    };
}

macro_rules! rotation_of {
    ($thing: expr) => {
        {
            let v = $thing.rotation.coords.data.to_vec();
            (v[0], v[1], v[2], v[3])
        }
    };
}

macro_rules! get_link_translation {
    ($link: ident) => {
           translation_of!($link.world_transform().expect(stringify!($link)))
    }
}

macro_rules! get_link_rotation {
    ($link: ident) => {
        {
           rotation_of!($link.world_transform().expect(stringify!($link)))
        }
    }
}

macro_rules! init_and_generate_data {
    (
        $fixed: ident, $l0: ident, $l1: ident, $l2: ident, $l3: ident,
        $arm: ident, $solver: ident, $constraints: ident,
        $target: ident, $angles: ident, $end: ident,
        $range: expr, $step: expr, $possible_targets: ident, $num: expr
    ) => {
        let mut data = String::from("");
        type translation = (f32, f32, f32);
        type rotation = (f32, f32, f32, f32);
        type row = (
            translation, translation, translation, translation,
            rotation, rotation, rotation, rotation,
            translation, translation, translation, translation,
            rotation, rotation, rotation, rotation,
            translation, translation, translation, translation,
            rotation, rotation, rotation, rotation,
            rotation, rotation
            );
        
        let mut rows = Vec::<row>::new();
        // let mut rows = Vec::<String>::new();

        init!(
            $fixed, $l0, $l1, $l2, $l3,
            $arm, $solver, $constraints, $target, $angles, $end
        );

        for (_x, _y, _z) in $possible_targets.iter() {
            $target.translation.vector[0] = *_x;
            $target.translation.vector[1] = *_y;
            $target.translation.vector[2] = *_z;
            
            match $solver.solve_with_constraints(&$arm, &$target, &$constraints) {
                Ok(_) => {$arm.update_transforms();},
                Err(_) => {continue;}
            };

            $range.clone().into_iter().for_each(|z| {
                $range.clone().into_iter().for_each(|y| {
                    $range.clone().into_iter().for_each(|x| {
                        $target.translation.vector[0] = *_x;
                        $target.translation.vector[1] = *_y;
                        $target.translation.vector[2] = *_z;
                        $arm.update_transforms();

                        let _l0 = get_link_translation!($l0); 
                        let _l1 = get_link_translation!($l1); 
                        let _l2 = get_link_translation!($l2); 
                        let _l3 = get_link_translation!($l3); 

                        let _l0_rot = get_link_rotation!($l0); 
                        let _l1_rot = get_link_rotation!($l1); 
                        let _l2_rot = get_link_rotation!($l2); 
                        let _l3_rot = get_link_rotation!($l3); 

                        let a_joint_pos = {
                                let v = $arm.joint_positions();
                                (v[0], v[1], v[2], v[3])
                            };

                        $target.translation.vector[0] += (x as f32) * $step;
                        $target.translation.vector[1] += (y as f32) * $step;  
                        $target.translation.vector[2] += (z as f32) * $step;

                        match $solver.solve_with_constraints(&$arm, &$target, &$constraints) {
                            Ok(_) => {
                                let transforms = $arm.update_transforms();

                                let l0_final = get_link_translation!($l0);
                                let l1_final = get_link_translation!($l1);
                                let l2_final = get_link_translation!($l2);
                                let l3_final = get_link_translation!($l3); 
                
                                let l0_final_rot = get_link_rotation!($l0); 
                                let l1_final_rot = get_link_rotation!($l1);
                                let l2_final_rot = get_link_rotation!($l2);
                                let l3_final_rot = get_link_rotation!($l3);

                                let l0_trans = translation_of!(transforms[0]);
                                let l1_trans = translation_of!(transforms[1]);
                                let l2_trans = translation_of!(transforms[2]);
                                let l3_trans = translation_of!(transforms[3]); 
                
                                let l0_trans_rot = rotation_of!(transforms[0]); 
                                let l1_trans_rot = rotation_of!(transforms[1]);
                                let l2_trans_rot = rotation_of!(transforms[2]);
                                let l3_trans_rot = rotation_of!(transforms[3]);

                                let b_joint_pos = {
                                        let v = $arm.joint_positions();
                                        (v[0], v[1], v[2], v[3])
                                    };

                                rows.push((
                                        _l0, _l1, _l2, _l3,
                                        _l0_rot, _l1_rot, _l2_rot, _l3_rot,
                                        l0_final, l1_final, l2_final, l3_final,
                                        l0_final_rot, l1_final_rot, l2_final_rot, l3_final_rot,
                                        l0_trans, l1_trans, l2_trans, l3_trans,
                                        l0_trans_rot, l1_trans_rot, l2_trans_rot, l3_trans_rot,
                                        a_joint_pos, b_joint_pos
                                ));
                            },
                            Err(_) => {}
                        }
                    });
                });
            });
        }

        rows.iter().for_each(|row| {
            let row_string = format!("{{
                                        \"l0\": {:?},\n
                                        \"l1\": {:?},\n
                                        \"l2\": {:?},\n
                                        \"l3\": {:?},\n
                                        \"l0_rot\": {:?},\n
                                        \"l1_rot\": {:?},\n
                                        \"l2_rot\": {:?},\n
                                        \"l3_rot\": {:?},\n
                                        \"l0_final\": {:?},\n
                                        \"l1_final\": {:?},\n
                                        \"l2_final\": {:?},\n
                                        \"l3_final\": {:?},\n
                                        \"l0_rot_final\": {:?},\n
                                        \"l1_rot_final\": {:?},\n
                                        \"l2_rot_final\": {:?},\n
                                        \"l3_rot_final\": {:?},\n
                                        \"l0_trans\": {:?},\n
                                        \"l1_trans\": {:?},\n
                                        \"l2_trans\": {:?},\n
                                        \"l3_trans\": {:?},\n
                                        \"l0_rot_trans\": {:?},\n
                                        \"l1_rot_trans\": {:?},\n
                                        \"l2_rot_trans\": {:?},\n
                                        \"l3_rot_trans\": {:?},\n
                                        \"a_joint_pos\": {:?},\n
                                        \"b_joint_pos\": {:?}\n
                                        }},\n
                                    ", row.0, row.1, row.2, row.3,
                                    row.4, row.5, row.6, row.7,
                                    row.8, row.9, row.10, row.11,
                                    row.12, row.13, row.14, row.15,
                                    row.16, row.17, row.18, row.19,
                                    row.20, row.21, row.22, row.23,
                                    row.24, row.25);
            data = format!("{}{}", data, row_string);
        });

        let mut f = File::create(format!("data{}.json", $num)).expect("Unable to create file");
        f.write_all(data.as_bytes()).expect("Unable to write data");

        println!("Thread {}'s work is done. See data{}.json", $num);
    }
}

fn main() {
    init!(
        fixed, l0, l1, l2, l3,
        arm, solver, constraints, target, angles, end
    );

    let mut possible_targets = Vec::<(f32, f32, f32)>::new();
    let range = -20..20;
    let step = 0.05;

    println!("Finding possible targets...");

    range.clone().into_iter().for_each(|z| {
        range.clone().into_iter().for_each(|y| {
            range.clone().into_iter().for_each(|x| {
                target.translation.vector[0] += (x as f32) * step;
                target.translation.vector[1] += (y as f32) * step;
                target.translation.vector[2] += (z as f32) * step;

                match solver.solve_with_constraints(&arm, &target, &constraints) {
                    Ok(_) => {
                            arm.update_transforms();
                            let t = l3.world_transform().expect("target 3").translation.vector.data.to_vec(); 
                            possible_targets.push((t[0], t[1], t[2]));
                    },
                    Err(_) => {}
                }
                arm.set_joint_positions(&angles).unwrap();
                arm.update_transforms();
                target = end.world_transform().unwrap().clone();                
            });
        });
    });

    println!("Found {} possible targets using a range of {:?}, and a step size of {}", possible_targets.len(), range, step);

    thread_rng().shuffle(&mut possible_targets);
    thread_rng().shuffle(&mut possible_targets);
    thread_rng().shuffle(&mut possible_targets);

    let (left, right) = possible_targets.split_at(possible_targets.len() / 2);
    let (first_q, second_q) = left.split_at(left.len() / 2);
    let (third_q, fourth_q) = left.split_at(right.len() / 2);
    
    let (l_first_q, r_first_q) = first_q.split_at(first_q.len() / 2);
    let (l_second_q, r_second_q) = second_q.split_at(first_q.len() / 2);
    let (l_third_q, r_third_q) = third_q.split_at(first_q.len() / 2);
    let (l_fourth_q, r_fourth_q) = fourth_q.split_at(first_q.len() / 2);

    rayon::scope(|s| {
        // 1 and 2
        s.spawn(|_| {
            init_and_generate_data! (
                    fixed0, l00, l10, l20, l30,
                    arm0, solver0, constraints0, target0, angles0, end0,
                    (-10..10), 0.1, l_first_q, 1
                );
        });

        s.spawn(|_| {
            init_and_generate_data! (
                    fixed1, l01, l11, l21, l31,
                    arm1, solver1, constraints1, target1, angles1, end1,
                    (-10..10), 0.1, r_first_q, 2
                );
        });

        // 3 and 4
        s.spawn(|_| {
            init_and_generate_data! (
                    fixed2, l02, l12, l22, l32,
                    arm2, solver2, constraints2, target2, angles2, end2,
                    (-10..10), 0.1, l_second_q, 3
                );
        });

        s.spawn(|_| {
            init_and_generate_data! (
                    fixed3, l03, l13, l23, l33,
                    arm3, solver3, constraints3, target3, angles3, end3,
                    (-10..10), 0.1, r_second_q, 4
                );
        });


        // 5 and 6
        s.spawn(|_| {
            init_and_generate_data! (
                    fixed0, l00, l10, l20, l30,
                    arm0, solver0, constraints0, target0, angles0, end0,
                    (-10..10), 0.1, l_third_q, 5
                );
        });

        s.spawn(|_| {
            init_and_generate_data! (
                    fixed1, l01, l11, l21, l31,
                    arm1, solver1, constraints1, target1, angles1, end1,
                    (-10..10), 0.1, r_third_q, 6
                );
        });


        // 7 and 8
        s.spawn(|_| {
            init_and_generate_data! (
                    fixed2, l02, l12, l22, l32,
                    arm2, solver2, constraints2, target2, angles2, end2,
                    (-10..10), 0.1, l_fourth_q, 7
                );
        });

        s.spawn(|_| {
            init_and_generate_data! (
                    fixed3, l03, l13, l23, l33,
                    arm3, solver3, constraints3, target3, angles3, end3,
                    (-10..10), 0.1, r_fourth_q, 8
                );
        });
    });

    let mut data = String::from("[");

    for i in 1..9 {
        data = format!("{}{}", data, read_to_string(format!("data{}.json", i))
        .expect("Something went wrong reading the file"));

        remove_file(
            format!("data{}.json", i)
        ).expect("Couldn't delete file");
    }

    data = format!("{}]", data);
    let mut f = File::create("DATA_12.json").expect("Unable to create file");
    f.write_all(data.as_bytes()).expect("Unable to write data");
}