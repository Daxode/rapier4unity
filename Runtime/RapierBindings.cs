using System;
using System.Runtime.InteropServices;
using Unity.Mathematics;
using UnityEngine;

namespace Packages.rapier4unity.Runtime
{
	public static class RapierBindings
	{
#if UNITY_STANDALONE_OSX
        private const string DllName = "librapier_c_bind.dylib";
#else
		private const string DllName = "rapier_c_bind";
#endif

		private const CallingConvention Convention = CallingConvention.Cdecl;

		// Native methods
		[DllImport(DllName, CallingConvention = Convention)]
		public static extern void init();

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern void teardown();

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern unsafe RawArray<CollisionEvent>* solve();

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern unsafe void free_collision_events(RawArray<CollisionEvent>* events);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern void set_gravity(float x, float y, float z);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern void set_time_step(float x);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern ColliderHandle add_cuboid_collider(float half_extents_x, float half_extents_y, float half_extents_z, float mass, bool is_sensor);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern ColliderHandle add_sphere_collider(float radius, float mass, bool is_sensor);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern ColliderHandle add_capsule_collider(float half_height, float radius, float mass, bool is_sensor);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern ColliderHandle add_mesh_collider([In] float[] vertices, int verticesCount, [In] uint[] indices, int indicesCount, float mass, bool is_sensor);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern ColliderHandle add_convex_mesh_collider([In] float[] vertices, int verticesCount, float mass, bool is_sensor);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern RigidBodyHandle add_rigid_body(
			ColliderHandle collider,
			RigidBodyType rigidBodyType,
			float position_x,
			float position_y,
			float position_z,
			float rotation_x,
			float rotation_y,
			float rotation_z,
			float rotation_w);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern RapierTransform get_transform(RigidBodyHandle rigidBodyHandle);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern RapierTransform set_transform_position(
			RigidBodyHandle rigidBodyHandle,
			float position_x,
			float position_y,
			float position_z);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern RapierTransform set_transform_rotation(
			RigidBodyHandle rigidBodyHandle,
			float rotation_x,
			float rotation_y,
			float rotation_z,
			float rotation_w);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern void set_linear_velocity(RigidBodyHandle handle, float velocity_x, float velocity_y, float velocity_z);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern void set_angular_velocity(RigidBodyHandle handle, float velocity_x, float velocity_y, float velocity_z);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern Vector3 get_linear_velocity(RigidBodyHandle handle);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern Vector3 get_angular_velocity(RigidBodyHandle handle);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern void enable_CCD(RigidBodyHandle handle, bool enabled);

		[DllImport(DllName, CallingConvention = Convention)]
		public static extern void add_force(RigidBodyHandle handle, float forceX, float forceY, float forceZ, ForceMode mode);

		[DllImport(DllName, CallingConvention = Convention)]
		private static extern unsafe bool cast_ray(float from_x, float from_y, float from_z, float dir_x, float dir_y, float dir_z, RapierRaycastHit* out_hit);

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
	}

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

	public enum RigidBodyType
	{
		Dynamic = 0,
		Fixed = 1,
		KinematicPositionBased = 2,
		KinematicVelocityBased = 3,
	}

	[StructLayout(LayoutKind.Sequential)]
	public struct RapierTransform
	{
		public quaternion rotation;
		public float3 position;

		public override string ToString() => $"Rotation: {rotation}, Position: {position}";
	}
}