using Codice.CM.Common;
using UnityEngine;

namespace RapierPhysics
{
    /// <summary>
    ///     The base class for all joints.
    /// </summary>
    public abstract class RapierJoint : MonoBehaviour
    {
        [Header("Joint Settings")]
        /// <summary>
        /// The anchor rigidbody of the joint in world space.
        /// </summary>
        /// 
        public Rigidbody Anchor;

        /// <summary>
        /// The articulation rigidbody that this joint is attached to.
        /// </summary>
        public Rigidbody Mover;

        /// <summary>
        /// The first anchor point in local space.
        /// </summary>
        public Vector3 Anchor1;

        /// <summary>
        /// The second anchor point in local space.
        /// </summary>
        public Vector3 Anchor2;

        /// <summary>
        /// The self collision flag. If false, the joint will not collide with itself.
        /// </summary>
        public bool SelfCollision = true;

        protected virtual void OnEnable()
        {
            // Only register the joint if both anchor and mover are set
            if (Anchor == null || Mover == null)
            {
                Debug.LogError("Anchor and Mover must be set for the joint to work properly.");
                return;
            }

            RapierLoop.RegisterJoint(this);
        }

        protected virtual void OnDisable()
        {
            RapierLoop.UnregisterJoint(this);
        }

        // Create gizmos to show the anchor points in the editor (only when selected)
        protected virtual void OnDrawGizmos()
        {
            if (Anchor == null || Mover == null) return;

            // Draw the anchor points in local space
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(Anchor.transform.TransformPoint(Anchor1), 0.1f);
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(Mover.transform.TransformPoint(Anchor2), 0.1f);
        }
    }
}