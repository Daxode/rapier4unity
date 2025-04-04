using System;
using System.Runtime.InteropServices;
using Unity.Mathematics;
using UnityEngine;

namespace Packages.rapier4unity.Runtime
{
	public struct RapierRaycastHit
	{
		internal Vector3 m_Point;
		internal Vector3 m_Normal;
		internal uint m_FaceID;
		internal float m_Distance;
		internal Vector2 m_UV;
		internal ColliderHandle m_Collider;
	}

	internal static class BindingExtensions
	{
		public static bool CastRay(float from_x, float from_y, float from_z, float dir_x, float dir_y, float dir_z, out RapierRaycastHit hit)
		{
			unsafe
			{
				RapierRaycastHit* hitPtr = stackalloc RapierRaycastHit[1];
				var did_hit = RapierBindings.CastRay(from_x, from_y, from_z, dir_x, dir_y, dir_z, hitPtr);
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
		internal uint index;
		internal uint generation;

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