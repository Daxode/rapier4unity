using System;
using System.Collections.Generic;
using Packages.rapier4unity.Runtime;
using RapierPhysics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Events;
using UnityEngine.LowLevel;
using Object = UnityEngine.Object;

namespace Packages.rapier4unity.Runtime
{
	/// <summary>
	/// This class is used to wrap the physics event system.
	/// Everything from OnTriggerEnter to OnJointBreak is wrapped here.
	/// </summary>
#if UNITY_EDITOR
	[UnityEditor.InitializeOnLoad]
#endif
	public static class PhysicsEventWrapper
	{
#if UNITY_EDITOR
		static PhysicsEventWrapper()
		{
			// Ensure that we teardown the physics bindings when exiting play mode
			UnityEditor.EditorApplication.playModeStateChanged += state =>
			{
				if (state == UnityEditor.PlayModeStateChange.EnteredEditMode)
				{
					if (RapierBindings.IsAvailable)
					{
						RapierBindings.Teardown();
						RapierBindings.UnloadCalls();
					}
				}
			};
		}
#endif

		[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
		static void SetupPlayerLoop()
		{
			RapierBindings.LoadCalls();

			// Get the current player loop
			PlayerLoopSystem loop = PlayerLoop.GetCurrentPlayerLoop();
			for (int i = 0; i < loop.subSystemList.Length; i++)
			{
				PlayerLoopSystem subSystem = loop.subSystemList[i];
				if (subSystem.type == typeof(UnityEngine.PlayerLoop.FixedUpdate))
				{
					// Append FixedUpdate to subsytems
					PlayerLoopSystem[] newSubSystems = new PlayerLoopSystem[subSystem.subSystemList.Length + 1];
					for (int j = 0; j < subSystem.subSystemList.Length; j++)
						newSubSystems[j] = subSystem.subSystemList[j];
					newSubSystems[subSystem.subSystemList.Length] = new PlayerLoopSystem
					{
						type = typeof(RapierLoop),
						updateDelegate = RapierLoop.FixedUpdate
					};
					subSystem.subSystemList = newSubSystems;
					loop.subSystemList[i] = subSystem;
				}

				if (subSystem.type == typeof(UnityEngine.PlayerLoop.Initialization))
				{
					// Append Rapier initialization to subsytems
					PlayerLoopSystem[] newSubSystems = new PlayerLoopSystem[subSystem.subSystemList.Length + 1];
					for (int j = 0; j < subSystem.subSystemList.Length; j++)
						newSubSystems[j] = subSystem.subSystemList[j];
					newSubSystems[subSystem.subSystemList.Length] = new PlayerLoopSystem
					{
						type = typeof(RapierLoop),
						updateDelegate = RapierLoop.Initialization
					};
					subSystem.subSystemList = newSubSystems;
					loop.subSystemList[i] = subSystem;
				}
			}

			// Set the modified player loop
			PlayerLoop.SetPlayerLoop(loop);
		}
	}
}


public class RapierLoop
{
	struct EventsForCollider
	{
		public List<BasicEvent<Collider>> onTriggerEnter;
		public List<BasicEvent<Collider>> onTriggerExit;
		public List<BasicEvent<Collider>> onTriggerStay;

		public List<BasicEvent<Collision>> onCollisionEnter;
		public List<BasicEvent<Collision>> onCollisionExit;
		public List<BasicEvent<Collision>> onCollisionStay;
	}

	struct EventsForMonoBehaviour
	{
		public BasicEvent<Collider> onTriggerEnter;
		public BasicEvent<Collider> onTriggerExit;
		public BasicEvent<Collider> onTriggerStay;

		public BasicEvent<Collision> onCollisionEnter;
		public BasicEvent<Collision> onCollisionExit;
		public BasicEvent<Collision> onCollisionStay;
	}

	struct BasicEvent<T>
	{
		//public delegate* <T, void> callback;
		delegate void Callback(T collider);
		Callback m_Callback;
		public void Invoke(T collider) => m_Callback?.Invoke(collider);
		public bool IsValid => m_Callback != null;
		public BasicEvent(MonoBehaviour component, string methodName)
		{
			System.Reflection.MethodInfo methodInfo = UnityEventBase.GetValidMethodInfo(component.GetType(), methodName, new[] { typeof(T) });
			m_Callback = (Callback)methodInfo?.CreateDelegate(typeof(Callback), component);
		}
	}

	static Dictionary<Rigidbody, RigidBodyHandle> rigidbodyToHandle = new();
	static Dictionary<Collider, ColliderHandle> colliderToHandle = new();
	static Dictionary<ColliderHandle, Collider> handleToCollider = new();
	static Dictionary<Collider, RigidBodyHandle> fixedRigidbodies = new();
	static Dictionary<Collider, EventsForCollider> physicsEvents = new();
	static Dictionary<RapierJoint, ImpulseJointHandle> joints = new();
	static Dictionary<MonoBehaviour, EventsForMonoBehaviour> monoBehaviourEvents = new();
	static HashSet<(Collider, Collider)> activeTriggerPairs = new();

	public static void RegisterJoint(RapierJoint joint)
	{
		if (joints.ContainsKey(joint))
			return;

		// If the mover is kinematic, we cannot continue
		if (joint.Mover.isKinematic)
		{
			Debug.LogError("Mover must not be kinematic for the joint to work properly.");
			return;
		}

		// Make sure the rigidbodies are registered
		if (!rigidbodyToHandle.ContainsKey(joint.Anchor))
		{
			RigidBodyHandle handle = CreateOrGetRigidBodyHandle(joint.Anchor);
			UpdateRigidBody(joint.Anchor, handle);
		}

		if (!rigidbodyToHandle.ContainsKey(joint.Mover))
		{
			RigidBodyHandle handle = CreateOrGetRigidBodyHandle(joint.Mover);
			UpdateRigidBody(joint.Mover, handle);
		}

		RigidBodyHandle anchorHandle = rigidbodyToHandle[joint.Anchor];
		RigidBodyHandle moverHandle = rigidbodyToHandle[joint.Mover];
		ImpulseJointHandle jointHandle;
		if (joint is RapierFixedJoint fixedJoint)
		{

			Vector3 anchor1 = Vector3.Scale(fixedJoint.Anchor1, fixedJoint.Anchor.transform.lossyScale);
			Vector3 anchor2 = Vector3.Scale(fixedJoint.Anchor2, fixedJoint.Mover.transform.lossyScale);
			jointHandle = RapierBindings.AddFixedJoint(
				anchorHandle,
				moverHandle,
				anchor1.x,
				anchor1.y,
				anchor1.z,
				anchor2.x,
				anchor2.y,
				anchor2.z,
				fixedJoint.SelfCollision);
		}
		else if (joint is RapierSphericalJoint sphericalJoint)
		{
			Vector3 anchor1 = Vector3.Scale(sphericalJoint.Anchor1, sphericalJoint.Anchor.transform.lossyScale);
			Vector3 anchor2 = Vector3.Scale(sphericalJoint.Anchor2, sphericalJoint.Mover.transform.lossyScale);
			jointHandle = RapierBindings.AddSphericalJoint(
				anchorHandle,
				moverHandle,
				anchor1.x,
				anchor1.y,
				anchor1.z,
				anchor2.x,
				anchor2.y,
				anchor2.z,
				sphericalJoint.SelfCollision);
		}
		// Check which kind of joint we need to create
		else if (joint is RapierRevoluteJoint revoluteJoint)
		{
			Vector3 anchor1 = Vector3.Scale(revoluteJoint.Anchor1, revoluteJoint.Anchor.transform.lossyScale);
			Vector3 anchor2 = Vector3.Scale(revoluteJoint.Anchor2, revoluteJoint.Mover.transform.lossyScale);
			jointHandle = RapierBindings.AddRevoluteJoint(
				anchorHandle,
				moverHandle,
				revoluteJoint.Axis.x,
				revoluteJoint.Axis.y,
				revoluteJoint.Axis.z,
				anchor1.x,
				anchor1.y,
				anchor1.z,
				anchor2.x,
				anchor2.y,
				anchor2.z,
				revoluteJoint.SelfCollision);
		}
		else if (joint is RapierPrismaticJoint prismaticJoint)
		{
			Vector3 anchor1 = Vector3.Scale(prismaticJoint.Anchor1, prismaticJoint.Anchor.transform.lossyScale);
			Vector3 anchor2 = Vector3.Scale(prismaticJoint.Anchor2, prismaticJoint.Mover.transform.lossyScale);
			jointHandle = RapierBindings.AddPrismaticJoint(
				anchorHandle,
				moverHandle,
				prismaticJoint.Axis.x,
				prismaticJoint.Axis.y,
				prismaticJoint.Axis.z,
				anchor1.x,
				anchor1.y,
				anchor1.z,
				anchor2.x,
				anchor2.y,
				anchor2.z,
				prismaticJoint.Limits.x,
				prismaticJoint.Limits.y,
				prismaticJoint.SelfCollision);
		}
		else
		{
			Debug.LogError($"Unsupported joint type: {joint.GetType()}");
			return;
		}

		joints[joint] = jointHandle;
	}

	public static void UnregisterJoint(RapierJoint joint)
	{
		if (!joints.ContainsKey(joint))
			return;

		// Remove joint handle
		ImpulseJointHandle jointHandle = joints[joint];
		RapierBindings.RemoveJoint(jointHandle);
		joints.Remove(joint);
	}

	public static void AddForceWithMode(Rigidbody rigidbody, Vector3 force, ForceMode mode)
	{
		RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
		RapierBindings.AddForce(handle, force.x, force.y, force.z, mode);
	}

	public static void AddForce(Rigidbody rigidbody, Vector3 force)
	{
		RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
		RapierBindings.AddForce(handle, force.x, force.y, force.z, ForceMode.Force);
	}

	public static void AddTorque(Rigidbody rigidbody, Vector3 torque)
	{
		RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
		RapierBindings.AddTorque(handle, torque.x, torque.y, torque.z, ForceMode.Force);
	}

	public static void AddTorqueWithMode(Rigidbody rigidbody, Vector3 torque, ForceMode mode)
	{
		RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
		RapierBindings.AddTorque(handle, torque.x, torque.y, torque.z, mode);
	}

	public static void MovePosition(Rigidbody rigidbody, Vector3 position)
	{
		if (!rigidbody.isKinematic)
		{
			Debug.LogError("MovePosition is not supported for non-kinematic rigidbodies. Apply forces instead or set the rigidbody to kinematic.");
			return;
		}

		RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
		RapierBindings.SetTransformPosition(handle, position.x, position.y, position.z);
	}

	public static void MoveRotation(Rigidbody rigidbody, Quaternion rotation)
	{
		if (!rigidbody.isKinematic)
		{
			Debug.LogError("MovePosition is not supported for non-kinematic rigidbodies. Apply forces instead or set the rigidbody to kinematic.");
			return;
		}

		RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
		RapierBindings.SetTransformRotation(handle, rotation.x, rotation.y, rotation.z, rotation.w);
	}

	public static void Move(Rigidbody rigidbody, Vector3 position, Quaternion rotation)
	{
		if (!rigidbody.isKinematic)
		{
			Debug.LogError("MovePosition is not supported for non-kinematic rigidbodies. Apply forces instead or set the rigidbody to kinematic.");
			return;
		}

		RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
		RapierBindings.SetTransform(handle, position.x, position.y, position.z, rotation.x, rotation.y, rotation.z, rotation.w);
	}

	public static void AddRelativeForce(Rigidbody rigidbody, Vector3 force)
	{
		throw new NotImplementedException("AddRelativeForce is not supported in Rapier4Unity. Use AddForce instead.");
	}

	public static void AddRelativeForceWithMode(Rigidbody rigidbody, Vector3 force, ForceMode mode)
	{
		throw new NotImplementedException("AddRelativeForce is not supported in Rapier4Unity. Use AddForce instead.");
	}

	public static void AddRelativeTorque(Rigidbody rigidbody, Vector3 torque)
	{
		throw new NotImplementedException("AddRelativeTorque is not supported in Rapier4Unity. Use AddTorque instead.");
	}

	public static void AddRelativeTorqueWithMode(Rigidbody rigidbody, Vector3 torque, ForceMode mode)
	{
		throw new NotImplementedException("AddRelativeTorque is not supported in Rapier4Unity. Use AddTorque instead.");
	}

	public struct LocalRaycastHit
	{
		internal Vector3 m_Point;
		internal Vector3 m_Normal;
		internal uint m_FaceID;
		internal float m_Distance;
		internal Vector2 m_UV;
		internal int m_Collider;
	}

	public static bool Raycast(Ray ray, out RaycastHit hit)
	{
		bool did_hit = BindingExtensions.CastRay(ray.origin.x, ray.origin.y, ray.origin.z, ray.direction.x, ray.direction.y, ray.direction.z, out RapierRaycastHit rapierHit);
		if (!did_hit)
		{
			hit = new RaycastHit();
			return false;
		}
		LocalRaycastHit localHit = new LocalRaycastHit
		{
			m_Point = rapierHit.m_Point,
			m_Normal = rapierHit.m_Normal,
			m_FaceID = rapierHit.m_FaceID,
			m_Distance = rapierHit.m_Distance,
			m_UV = rapierHit.m_UV,
			m_Collider = handleToCollider[rapierHit.m_Collider].GetInstanceID()
		};
		hit = UnsafeUtility.As<LocalRaycastHit, RaycastHit>(ref localHit);
		return true;
	}


	// Called in the beginning of every frame, this ensures that all colliders and rigidbodies are initialized
	public static unsafe void Initialization()
	{
		if (!Application.isPlaying || !RapierBindings.IsAvailable)
			return;

		// Find and add all colliders
		foreach (Collider collider in Object.FindObjectsByType<Collider>(FindObjectsSortMode.None))
		{
			AddCollider(collider);
		}

		// Find and add all rigidbodies
		foreach (Rigidbody rigidbody in Object.FindObjectsByType<Rigidbody>(FindObjectsSortMode.None))
		{
			RigidBodyHandle handle = CreateOrGetRigidBodyHandle(rigidbody);
			UpdateRigidBody(rigidbody, handle);
		}
	}

	private unsafe static void AddCollider(Collider collider)
	{
		// Add Rapier Collider if it doesn't exist
		if (!colliderToHandle.ContainsKey(collider))
		{
			Rigidbody potentialRigidbody = collider.GetComponent<Rigidbody>();
			Vector3 transformScale = collider.transform.localScale;

			switch (collider)
			{
				case BoxCollider boxCollider:
					{
						ColliderHandle newColliderHandle = RapierBindings.AddCuboidCollider(
							transformScale.x * boxCollider.size.x / 2,
							transformScale.y * boxCollider.size.y / 2,
							transformScale.z * boxCollider.size.z / 2,
							potentialRigidbody == null ? 0 : potentialRigidbody.mass,
							boxCollider.isTrigger);

						colliderToHandle[collider] = newColliderHandle;
						handleToCollider[newColliderHandle] = collider;
						break;
					}
				case SphereCollider sphereCollider:
					{
						ColliderHandle newColliderHandle = RapierBindings.AddSphereCollider(
							transformScale.x * sphereCollider.radius,
							potentialRigidbody == null ? 0 : potentialRigidbody.mass,
							sphereCollider.isTrigger);
						colliderToHandle[collider] = newColliderHandle;
						handleToCollider[newColliderHandle] = collider;
						break;
					}
				case CapsuleCollider capsuleCollider:
					{
						ColliderHandle newColliderHandle = RapierBindings.AddCapsuleCollider(
							transformScale.x * capsuleCollider.radius,
							transformScale.y * (capsuleCollider.height / 2) - (transformScale.x * capsuleCollider.radius),
							potentialRigidbody == null ? 0 : potentialRigidbody.mass,
							capsuleCollider.isTrigger);
						colliderToHandle[collider] = newColliderHandle;
						handleToCollider[newColliderHandle] = collider;
						break;
					}
				case MeshCollider meshCollider:
					{
						// Make sure we have a valid mesh
						Mesh mesh = meshCollider.sharedMesh;
						if (mesh == null)
						{
							Debug.LogError($"MeshCollider on {collider.gameObject.name} has no mesh assigned!");
							return;
						}

						// Get vertex data
						Vector3[] vertices = mesh.vertices;
						Vector3 scale = collider.transform.localScale;
						NativeArray<float3> verticesFlat = new NativeArray<float3>(vertices.Length, Allocator.Temp);

						// Apply local scale to vertices
						for (int i = 0; i < vertices.Length; i++)
							verticesFlat[i] = (float3)vertices[i] * scale;

						ColliderHandle newColliderHandle;

						if (meshCollider.convex)
						{
							// Use convex hull for convex meshes (better performance)
							newColliderHandle = RapierBindings.AddConvexMeshCollider(
								(float*)verticesFlat.GetUnsafeReadOnlyPtr(), (UIntPtr)vertices.Length,
								potentialRigidbody == null ? 0 : potentialRigidbody.mass,
								meshCollider.isTrigger);
						}
						else
						{
							// Use trimesh for concave meshes
							int[] triangles = mesh.triangles;
							var indicesFlat = new NativeArray<uint>(triangles.Length, Allocator.Temp);
							for (int i = 0; i < triangles.Length; i++)
								indicesFlat[i] = (uint)triangles[i];


							newColliderHandle = RapierBindings.AddMeshCollider(
								(float*)verticesFlat.GetUnsafeReadOnlyPtr(), (UIntPtr)vertices.Length,
								(uint*)indicesFlat.GetUnsafeReadOnlyPtr(), (UIntPtr)(triangles.Length / 3),
								potentialRigidbody == null ? 0 : potentialRigidbody.mass,
								meshCollider.isTrigger);
						}

						colliderToHandle[collider] = newColliderHandle;
						handleToCollider[newColliderHandle] = collider;
						break;
					}
			}

			// Add events for collider
			EventsForCollider eventsForCollider = new EventsForCollider();

			eventsForCollider.onTriggerEnter = new List<BasicEvent<Collider>>();
			eventsForCollider.onTriggerExit = new List<BasicEvent<Collider>>();
			eventsForCollider.onTriggerStay = new List<BasicEvent<Collider>>();

			eventsForCollider.onCollisionEnter = new List<BasicEvent<Collision>>();
			eventsForCollider.onCollisionExit = new List<BasicEvent<Collision>>();
			eventsForCollider.onCollisionStay = new List<BasicEvent<Collision>>();

			MonoBehaviour[] components = collider.gameObject.GetComponents<MonoBehaviour>();
			foreach (MonoBehaviour component in components)
			{
				// Lookup events for MonoBehaviour, create if they don't exist
				if (!monoBehaviourEvents.TryGetValue(component, out EventsForMonoBehaviour eventsForMonoBehaviour))
				{
					eventsForMonoBehaviour.onTriggerEnter = new BasicEvent<Collider>(component, "OnTriggerEnter");
					eventsForMonoBehaviour.onTriggerExit = new BasicEvent<Collider>(component, "OnTriggerExit");
					eventsForMonoBehaviour.onTriggerStay = new BasicEvent<Collider>(component, "OnTriggerStay");

					eventsForMonoBehaviour.onCollisionEnter = new BasicEvent<Collision>(component, "OnCollisionEnter");
					eventsForMonoBehaviour.onCollisionExit = new BasicEvent<Collision>(component, "OnCollisionExit");
					eventsForMonoBehaviour.onCollisionStay = new BasicEvent<Collision>(component, "OnCollisionStay");

					monoBehaviourEvents[component] = eventsForMonoBehaviour;
				}

				// Add events for collider
				if (eventsForMonoBehaviour.onTriggerEnter.IsValid)
					eventsForCollider.onTriggerEnter.Add(eventsForMonoBehaviour.onTriggerEnter);
				if (eventsForMonoBehaviour.onTriggerExit.IsValid)
					eventsForCollider.onTriggerExit.Add(eventsForMonoBehaviour.onTriggerExit);
				if (eventsForMonoBehaviour.onTriggerStay.IsValid)
					eventsForCollider.onTriggerStay.Add(eventsForMonoBehaviour.onTriggerStay);

				if (eventsForMonoBehaviour.onCollisionEnter.IsValid)
					eventsForCollider.onCollisionEnter.Add(eventsForMonoBehaviour.onCollisionEnter);
				if (eventsForMonoBehaviour.onCollisionExit.IsValid)
					eventsForCollider.onCollisionExit.Add(eventsForMonoBehaviour.onCollisionExit);
				if (eventsForMonoBehaviour.onCollisionStay.IsValid)
					eventsForCollider.onCollisionStay.Add(eventsForMonoBehaviour.onCollisionStay);
			}

			physicsEvents[collider] = eventsForCollider;

			// In Unity if an object doesn't have a rigidbody, it's considered static
			// In Rapier, we need to add a fixed rigidbody, so we can simulate dynamic object interacting with static objects
			if (potentialRigidbody == null && colliderToHandle.TryGetValue(collider, out ColliderHandle colliderHandle))
			{
				fixedRigidbodies[collider] = RapierBindings.AddRigidBody(
					colliderHandle,
					RigidBodyType.Fixed,
					collider.transform.position.x,
					collider.transform.position.y,
					collider.transform.position.z,
					collider.transform.rotation.x,
					collider.transform.rotation.y,
					collider.transform.rotation.z,
					collider.transform.rotation.w);
			}
		}
	}

	private static RigidBodyHandle CreateOrGetRigidBodyHandle(Rigidbody rigidbody, RigidBodyType type = RigidBodyType.Dynamic)
	{
		// Try to get the handle from the dictionary first
		if (rigidbodyToHandle.TryGetValue(rigidbody, out RigidBodyHandle handle))
		{
			return handle;
		}

		// Handle doesn't exist, create a new one
		Collider[] colliders = rigidbody.GetComponents<Collider>();
		Assert.AreEqual(colliders.Length, 1, "Rigidbody must have exactly one collider for the moment");

		// Try to add a collider in case we don't have one yet
		AddCollider(colliders[0]);
		ColliderHandle colliderHandle = colliderToHandle[colliders[0]];
		Transform trs = rigidbody.transform;
		RigidBodyHandle rigidBodyHandle = RapierBindings.AddRigidBody(
			colliderHandle,
			type,
			trs.position.x,
			trs.position.y,
			trs.position.z,
			trs.rotation.x,
			trs.rotation.y,
			trs.rotation.z,
			trs.rotation.w);

		rigidbodyToHandle[rigidbody] = rigidBodyHandle;
		return rigidBodyHandle;
	}

	private static void UpdateRigidBody(Rigidbody rigidbody, RigidBodyHandle handle)
	{
		// TODO Determine what other kinds of properties might need to update per frame. 
		RapierBindings.UpdateRigidBodyProperties(handle,
		rigidbody.isKinematic ? RigidBodyType.KinematicPositionBased : RigidBodyType.Dynamic,
		rigidbody.collisionDetectionMode == CollisionDetectionMode.Continuous,
		(uint)rigidbody.constraints,
		rigidbody.linearDamping,
		rigidbody.angularDamping);
	}

	// Called at the end of the FixedUpdate loop
	// This is where we solve physics and get collision events
	// After that we update the GameObject positions of GameObjects with RigidBody component
	public static void FixedUpdate()
	{
		// Only run in play mode
		if (!Application.isPlaying || !RapierBindings.IsAvailable)
			return;

		unsafe
		{
			// Solve physics and get collision events
			RawArray<CollisionEvent>* eventsPtrToArray = RapierBindings.Solve();
			if (eventsPtrToArray == null)
				return;

			// Handle collision events
			for (int i = 0; i < eventsPtrToArray->length; i++)
			{
				CollisionEvent @event = (*eventsPtrToArray)[i];
				Collider collider1 = handleToCollider[@event.collider1];
				Collider collider2 = handleToCollider[@event.collider2];
				EventsForCollider eventsForCollider1 = physicsEvents[collider1];
				EventsForCollider eventsForCollider2 = physicsEvents[collider2];
				if (@event.is_started)
				{
					if (collider1.isTrigger || collider2.isTrigger)
					{
						foreach (BasicEvent<Collider> enter in eventsForCollider1.onTriggerEnter)
							enter.Invoke(collider2);
						foreach (BasicEvent<Collider> enter in eventsForCollider2.onTriggerEnter)
							enter.Invoke(collider1);
					}

					activeTriggerPairs.Add((collider1, collider2));
				}
				else
				{
					if (collider1.isTrigger || collider2.isTrigger)
					{
						foreach (BasicEvent<Collider> exit in eventsForCollider1.onTriggerExit)
							exit.Invoke(collider2);
						foreach (BasicEvent<Collider> exit in eventsForCollider2.onTriggerExit)
							exit.Invoke(collider1);
					}
					activeTriggerPairs.Remove((collider1, collider2));
				}
			}

			// Handle trigger stay
			foreach ((Collider, Collider) activeTriggerPair in activeTriggerPairs)
			{
				(Collider collider1, Collider collider2) = activeTriggerPair;
				EventsForCollider eventsForCollider1 = physicsEvents[collider1];
				EventsForCollider eventsForCollider2 = physicsEvents[collider2];
				if (collider1.isTrigger || collider2.isTrigger)
				{
					foreach (BasicEvent<Collider> stay in eventsForCollider1.onTriggerStay)
						stay.Invoke(collider2);
					foreach (BasicEvent<Collider> stay in eventsForCollider2.onTriggerStay)
						stay.Invoke(collider1);
				}
			}
			RapierBindings.FreeCollisionEvents(eventsPtrToArray);
		}

		// Update GameObject positions of GameObjects with RigidBody component
		foreach (Rigidbody rigidbody in Object.FindObjectsByType<Rigidbody>(FindObjectsSortMode.None))
		{
			RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
			RapierTransform position = RapierBindings.GetTransform(handle);
			rigidbody.transform.SetPositionAndRotation(position.position, position.rotation);
		}

		// Update fixed rigidbodies
		foreach ((Collider collider, RigidBodyHandle rbhandle) in fixedRigidbodies)
		{
		}
	}
}