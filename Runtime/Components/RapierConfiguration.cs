using System;
using UnityEngine;

namespace RapierPhysics
{
    public class RapierConfiguration : MonoBehaviour
    {
        [Header("Time Settings")]
        [Tooltip("The ticks per second for physics simulation. Default: 50 (1.0f/50.0f = 0.02 seconds)")]
        public float PhysicsTicksPerSecond = 50.0f;

        [Header("Solver Parameters")]
        [Tooltip("The number of solver iterations run by the constraints solver for calculating forces. Higher values increase accuracy but decrease performance. Default: 4")]
        public int NumSolverIterations = 4;

        [Tooltip("Number of internal Project Gauss Seidel (PGS) iterations run at each solver iteration. Default: 1")]
        public int NumInternalPgsIterations = 1;

        [Tooltip("Number of additional friction resolution iterations run during the last solver sub-step. Default: 0")]
        public int NumAdditionalFrictionIterations = 0;

        [Tooltip("The number of stabilization iterations run at each solver iteration. Default: 2")]
        public int NumInternalStabilizationIterations = 2;

        [Tooltip("Maximum number of substeps performed by the continuous collision detection (CCD) solver. Higher values improve collision accuracy but increase computational cost. Default: 1")]
        public int MaxCcdSubsteps = 1;

        [Header("Constraint Parameters")]
        [Tooltip("The damping ratio used by the springs for contact constraint stabilization. Larger values make the constraints more compliant (allowing more visible penetrations before stabilization). Default: 5.0")]
        public float ContactDampingRatio = 5.0f;

        [Tooltip("The natural frequency used by the springs for contact constraint regularization. Increasing this value will make penetrations get fixed more quickly at the expense of potential jitter effects due to overshooting. Default: 30.0")]
        public float ContactNaturalFrequency = 30.0f;

        [Tooltip("The natural frequency used by the springs for joint constraint regularization. Increasing this value will make penetrations get fixed more quickly. Default: 1.0e6")]
        public float JointNaturalFrequency = 1.0e6f;

        [Tooltip("The fraction of critical damping applied to joints for constraints regularization. Larger values make the constraints more compliant (allowing more joint drift before stabilization). Default: 1.0")]
        public float JointDampingRatio = 1.0f;

        [Header("Collision and Correction")]
        [Tooltip("The approximate size of most dynamic objects in the scene, representing units-per-meter. This value is used internally to scale various length-based tolerances. For example, in a 2D game, if your typical object size is 100 pixels, set this to 100.0. Default: 1.0")]
        public float LengthUnit = 1.0f;

        [Tooltip("The maximal distance separating two objects that will generate predictive contacts. This value is implicitly scaled by length_unit. Default: 0.002m")]
        public float NormalizedPredictionDistance = 0.002f;

        [Tooltip("Maximum amount of penetration the solver will attempt to resolve in one timestep. This value is implicitly scaled by length_unit. Default: 10.0")]
        public float NormalizedMaxCorrectiveVelocity = 10.0f;


        public void Awake()
        {
            RapierBindings.SetIntegrationParameters(
                1.0f / PhysicsTicksPerSecond,
                new UIntPtr((uint)NumSolverIterations),
                new UIntPtr((uint)NumInternalPgsIterations),
                new UIntPtr((uint)NumAdditionalFrictionIterations),
                new UIntPtr((uint)NumInternalStabilizationIterations),
                new UIntPtr((uint)MaxCcdSubsteps),
                ContactDampingRatio,
                JointDampingRatio,
                ContactNaturalFrequency,
                JointNaturalFrequency,
                NormalizedPredictionDistance,
                NormalizedMaxCorrectiveVelocity,
                LengthUnit
            );
        }
    }
}