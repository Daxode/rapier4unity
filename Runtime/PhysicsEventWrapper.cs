using System;
using System.Collections.Generic;
using Packages.rapier4unity.Runtime;
using Unity.Collections.LowLevel.Unsafe;
using UnityEditor;
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
	[InitializeOnLoad]
	public static class PhysicsEventWrapper
	{
		static PhysicsEventWrapper()
		{
			RapierBindings.init();
			AssemblyReloadEvents.beforeAssemblyReload += RapierBindings.teardown;

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
	static Dictionary<MonoBehaviour, EventsForMonoBehaviour> monoBehaviourEvents = new();
	static HashSet<(Collider, Collider)> activeTriggerPairs = new();

	public static void AddForceWithMode(Rigidbody rigidbody, Vector3 force, ForceMode mode)
	{
		RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
		RapierBindings.add_force(handle, force.x, force.y, force.z, mode);
	}

	public static void AddForce(Rigidbody rigidbody, Vector3 force)
	{
		RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
		RapierBindings.add_force(handle, force.x, force.y, force.z, ForceMode.Force);
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
		bool did_hit = RapierBindings.cast_ray(ray.origin.x, ray.origin.y, ray.origin.z, ray.direction.x, ray.direction.y, ray.direction.z, out RapierBindings.RapierRaycastHit rapierHit);
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
	public static void Initialization()
	{
		if (!Application.isPlaying)
			return;

		foreach (Collider collider in Object.FindObjectsByType<Collider>(FindObjectsSortMode.None))
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
							ColliderHandle newColliderHandle = RapierBindings.add_cuboid_collider(
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
							ColliderHandle newColliderHandle = RapierBindings.add_sphere_collider(
								transformScale.x * sphereCollider.radius,
								potentialRigidbody == null ? 0 : potentialRigidbody.mass,
								sphereCollider.isTrigger);
							colliderToHandle[collider] = newColliderHandle;
							handleToCollider[newColliderHandle] = collider;
							break;
						}
					case CapsuleCollider capsuleCollider:
						{
							ColliderHandle newColliderHandle = RapierBindings.add_capsule_collider(
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
								continue;
							}

							// Get vertex data
							Vector3[] vertices = mesh.vertices;
							Vector3 scale = collider.transform.localScale;
							float[] verticesFlat = new float[vertices.Length * 3];

							// Apply local scale to vertices
							for (int i = 0; i < vertices.Length; i++)
							{
								verticesFlat[i * 3] = vertices[i].x * scale.x;
								verticesFlat[i * 3 + 1] = vertices[i].y * scale.y;
								verticesFlat[i * 3 + 2] = vertices[i].z * scale.z;
							}

							ColliderHandle newColliderHandle;

							if (meshCollider.convex)
							{
								// Use convex hull for convex meshes (better performance)
								newColliderHandle = RapierBindings.add_convex_mesh_collider(
									verticesFlat, vertices.Length,
									potentialRigidbody == null ? 0 : potentialRigidbody.mass,
									meshCollider.isTrigger);
							}
							else
							{
								// Use trimesh for concave meshes
								int[] triangles = mesh.triangles;
								uint[] indicesFlat = new uint[triangles.Length];
								for (int i = 0; i < triangles.Length; i++)
								{
									indicesFlat[i] = (uint)triangles[i];
								}

								newColliderHandle = RapierBindings.add_mesh_collider(
									verticesFlat, vertices.Length,
									indicesFlat, triangles.Length / 3,
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
					fixedRigidbodies[collider] = RapierBindings.add_rigid_body(
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

		foreach (Rigidbody rigidbody in Object.FindObjectsByType<Rigidbody>(FindObjectsSortMode.None))
		{
			// Add Rapier RigidBody if it doesn't exist
			if (!rigidbodyToHandle.ContainsKey(rigidbody))
			{
				Collider[] colliders = rigidbody.GetComponents<Collider>();
				Assert.AreEqual(colliders.Length, 1, "Rigidbody must have exactly one collider for the moment");
				ColliderHandle colliderHandle = colliderToHandle[colliders[0]];
				Transform trs = rigidbody.transform;
				rigidbodyToHandle[rigidbody] = RapierBindings.add_rigid_body(
					colliderHandle,
					RigidBodyType.Dynamic,
					trs.position.x,
					trs.position.y,
					trs.position.z,
					trs.rotation.x,
					trs.rotation.y,
					trs.rotation.z,
					trs.rotation.w);
			}
			RapierBindings.enable_CCD(rigidbodyToHandle[rigidbody], rigidbody.collisionDetectionMode == CollisionDetectionMode.Continuous);
		}

	}

	// Called at the end of the FixedUpdate loop
	// This is where we solve physics and get collision events
	// After that we update the GameObject positions of GameObjects with RigidBody component
	public static void FixedUpdate()
	{
		// Only run in play mode
		if (!Application.isPlaying)
			return;

		unsafe
		{
			// Solve physics and get collision events
			RawArray<CollisionEvent>* eventsPtrToArray = RapierBindings.solve();

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
			RapierBindings.free_collision_events(eventsPtrToArray);
		}

		// Update GameObject positions of GameObjects with RigidBody component
		foreach (Rigidbody rigidbody in Object.FindObjectsByType<Rigidbody>(FindObjectsSortMode.None))
		{
			RigidBodyHandle handle = rigidbodyToHandle[rigidbody];
			RapierTransform position = RapierBindings.get_transform(handle);
			rigidbody.transform.SetPositionAndRotation(position.position, position.rotation);
		}

		// Update fixed rigidbodies
		foreach ((Collider collider, RigidBodyHandle rbhandle) in fixedRigidbodies)
		{

		}
	}
}