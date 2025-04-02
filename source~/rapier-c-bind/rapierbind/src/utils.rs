use rapier3d::prelude::{LockedAxes, RigidBody};

/// Converts Unity RigidbodyConstraints enum value to Rapier LockedAxes
pub fn unity_constraints_to_locked_axes(constraints: u32) -> LockedAxes {
    // Unity's constraints are shifted 1 bit left compared to our LockedAxes
    // We need to map:
    // - bits 1,2,3 (positions) → 0,1,2
    // - bits 4,5,6 (rotations) → 3,4,5

    // Extract the position flags (bits 1-3) and shift right
    let pos_flags = (constraints & 0b1110) >> 1;

    // Extract the rotation flags (bits 4-6) and shift right
    let rot_flags = (constraints & 0b1111000) >> 1;

    // Combine the flags
    LockedAxes::from_bits_truncate((pos_flags | rot_flags) as u8)
}

/// Converts Rapier LockedAxes to Unity RigidbodyConstraints enum value
pub fn locked_axes_to_unity_constraints(locked_axes: LockedAxes) -> u32 {
    // Convert LockedAxes to Unity constraints by shifting left
    // - bits 0,1,2 (translation) → 1,2,3
    // - bits 3,4,5 (rotation) → 4,5,6

    // Get the raw bits
    let bits = locked_axes.bits() as u32;

    // Shift left by 1 to match Unity's bit positions
    bits << 1
}

pub fn cancel_axis_velocity(current_locks: LockedAxes, rigidbody: &mut RigidBody) {
    // Cancel the velocity for locked translation axes
    let mut linvel = *rigidbody.linvel();

    if current_locks.contains(LockedAxes::TRANSLATION_LOCKED_X) {
        linvel[0] = 0.0;
    }
    if current_locks.contains(LockedAxes::TRANSLATION_LOCKED_Y) {
        linvel[1] = 0.0;
    }
    if current_locks.contains(LockedAxes::TRANSLATION_LOCKED_Z) {
        linvel[2] = 0.0;
    }
    rigidbody.set_linvel(linvel, false);

    // Cancel the velocity for locked rotation axes
    let mut angvel = *rigidbody.angvel();
    if current_locks.contains(LockedAxes::ROTATION_LOCKED_X) {
        angvel[0] = 0.0;
    }
    if current_locks.contains(LockedAxes::ROTATION_LOCKED_Y) {
        angvel[1] = 0.0;
    }
    if current_locks.contains(LockedAxes::ROTATION_LOCKED_Z) {
        angvel[2] = 0.0;
    }
    rigidbody.set_angvel(angvel, false);
}
