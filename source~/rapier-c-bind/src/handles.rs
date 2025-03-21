use rapier3d::prelude::*;

// CollHandle is a handle to a collider.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SerializableColliderHandle {
    index: u32,
    generation: u32,
}

impl From<ColliderHandle> for SerializableColliderHandle {
    fn from(value: ColliderHandle) -> Self {
        let val = value.into_raw_parts();
        SerializableColliderHandle {
            index: val.0,
            generation: val.1
        }
    }
}

impl From<SerializableColliderHandle> for ColliderHandle {
    fn from(value: SerializableColliderHandle) -> Self {
        ColliderHandle::from_raw_parts(value.index, value.generation)
    }
}


// RigidBodyHandle is a handle to a rigid body.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct SerializableRigidBodyHandle {
    index: u32,
    generation: u32,
}

impl From<RigidBodyHandle> for SerializableRigidBodyHandle {
    fn from(value: RigidBodyHandle) -> Self {
        let val = value.into_raw_parts();
        SerializableRigidBodyHandle {
            index: val.0,
            generation: val.1
        }
    }
}

impl From<SerializableRigidBodyHandle> for RigidBodyHandle {
    fn from(value: SerializableRigidBodyHandle) -> Self {
        RigidBodyHandle::from_raw_parts(value.index, value.generation)
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub enum SerializableRigidBodyType {
    Dynamic = 0,
    Fixed = 1,
    KinematicPositionBased = 2,
    KinematicVelocityBased = 3,
}

impl From<RigidBodyType> for SerializableRigidBodyType {
    fn from(value: RigidBodyType) -> Self {
        match value {
            RigidBodyType::Dynamic => SerializableRigidBodyType::Dynamic,
            RigidBodyType::Fixed => SerializableRigidBodyType::Fixed,
            RigidBodyType::KinematicPositionBased => SerializableRigidBodyType::KinematicPositionBased,
            RigidBodyType::KinematicVelocityBased => SerializableRigidBodyType::KinematicVelocityBased,
        }
    }
}

impl From<SerializableRigidBodyType> for RigidBodyType {
    fn from(value: SerializableRigidBodyType) -> Self {
        match value {
            SerializableRigidBodyType::Dynamic => RigidBodyType::Dynamic,
            SerializableRigidBodyType::Fixed => RigidBodyType::Fixed,
            SerializableRigidBodyType::KinematicPositionBased => RigidBodyType::KinematicPositionBased,
            SerializableRigidBodyType::KinematicVelocityBased => RigidBodyType::KinematicVelocityBased,
        }
    }
}