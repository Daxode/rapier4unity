using UnityEngine;

namespace RapierPhysics
{
    public class RapierRevoluteJoint : RapierJoint
    {
        /// <summary>
        /// The axis of rotation in local space.
        /// </summary>
        public Vector3 Axis;

        // Override the gizmos to show the axis of rotation
        protected override void OnDrawGizmos()
        {
            base.OnDrawGizmos();
            if (Anchor == null || Mover == null) return;

            Gizmos.color = Color.red;
            Gizmos.DrawLine(Anchor.transform.TransformPoint(Anchor1), Anchor.transform.TransformPoint(Anchor1 + Axis));
        }
    }
}