using System.Runtime.InteropServices;
using Unity.Mathematics;
using UnityEngine;
using System;

namespace Packages.rapier4unity.Runtime
{
    [StructLayout(LayoutKind.Sequential)]
    public struct CollisionEvent
    {
        public ColliderHandle collider1;
        public ColliderHandle collider2;
        public bool is_started;
    }
    
    public struct RawArray<T> where T : unmanaged
    {
        public IntPtr data;
        public int length;
        public int capacity;
        
        public T this[int index]
        {
            get
            {
                unsafe
                {
                    if (index < 0 || index >= length)
                        throw new IndexOutOfRangeException($"Index {index} is out of range [0, {length})");
                    if ((T*)data == null)
                        throw new NullReferenceException("The array is not initialized");
                    return ((T*)data)[index];
                }
            }
        }
    }
    
    public class RapierBindings
    {
        #if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        #else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
        #endif
        public static extern void init();
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern void teardown();
        
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern unsafe RawArray<CollisionEvent>* solve();
        
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern unsafe void free_collision_events(RawArray<CollisionEvent>* events);
        
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern void set_gravity(float x, float y, float z);
        
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern void set_time_step(float x);
        
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern ColliderHandle add_cuboid_collider(float half_extents_x, float half_extents_y, float half_extents_z, float mass, bool is_sensor);
        
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern ColliderHandle add_sphere_collider(float radius, float mass, bool is_sensor);
        
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern RigidBodyHandle add_rigid_body(ColliderHandle collider, RigidBodyType rigidBodyType, float position_x, float position_y, float position_z);
        
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern RapierPosition get_position(RigidBodyHandle rigidBodyHandle);

#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern void add_force(RigidBodyHandle handle, float forceX, float forceY, float forceZ, ForceMode mode);

        public struct RapierRaycastHit
        {
            internal Vector3 m_Point;
            internal Vector3 m_Normal;
            internal uint m_FaceID;
            internal float m_Distance;
            internal Vector2 m_UV;
            internal ColliderHandle m_Collider;
        }

        public static bool cast_ray(float from_x, float from_y, float from_z, float dir_x, float dir_y, float dir_z, out RapierRaycastHit hit)
        {
            unsafe
            {
                RapierRaycastHit* hitPtr = stackalloc RapierRaycastHit[1];
                var did_hit = cast_ray(from_x, from_y, from_z, dir_x, dir_y, dir_z, hitPtr);
                hit = *hitPtr;
                return did_hit;
            }
        }
        
#if UNITY_STANDALONE_OSX
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
#else
        [DllImport("rapier_c_bind", CallingConvention = CallingConvention.Cdecl)]
#endif
        public static extern unsafe bool cast_ray(float from_x, float from_y, float from_z, float dir_x, float dir_y, float dir_z, RapierRaycastHit* out_hit);
    }
    
    public struct ColliderHandle
    {
        uint index;
        uint generation;

        public override string ToString() => $"Index: {index}, Generation: {generation}";
    }
    
    public struct RigidBodyHandle
    {
        uint index;
        uint generation;
        
        public override string ToString() => $"Index: {index}, Generation: {generation}";
    }
    
    public enum RigidBodyType {
        Dynamic = 0,
        Fixed = 1,
        KinematicPositionBased = 2,
        KinematicVelocityBased = 3,
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct RapierPosition
    {
        public quaternion rotation;
        public float3 position;
        
        public override string ToString() => $"Rotation: {rotation}, Position: {position}";
    }
}
