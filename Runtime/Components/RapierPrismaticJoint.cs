using UnityEngine;

namespace RapierPhysics
{
    public class RapierPrismaticJoint : RapierJoint
    {
        /// <summary>
        /// The axis of rotation in local space.
        /// </summary>
        public Vector3 Axis = Vector3.up;

        /// <summary>
        /// The limits of the joint in local space.
        /// </summary>
        public Vector2 Limits = new Vector2(-1, 1);

        // Override the gizmos to show the axis of rotation and limits
        protected override void OnDrawGizmos()
        {
            base.OnDrawGizmos();
            if (Anchor == null || Mover == null) return;

            Gizmos.color = Color.red;
            Gizmos.DrawLine(Anchor.transform.TransformPoint(Anchor1), Anchor.transform.TransformPoint(Anchor1 + Axis));

            Gizmos.color = Color.green;
            Vector3 limit1 = Anchor.transform.TransformPoint(Anchor1 + Axis * Limits.x);
            Vector3 limit2 = Anchor.transform.TransformPoint(Anchor1 + Axis * Limits.y);
            Gizmos.DrawLine(limit1, limit2);
            Gizmos.DrawSphere(limit1, 0.1f);
            Gizmos.DrawSphere(limit2, 0.1f);
        }
    }
}