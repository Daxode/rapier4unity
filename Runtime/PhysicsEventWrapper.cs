using System;
using System.Collections.Generic;
using Packages.rapier4unity.Runtime;
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
	        var loop = PlayerLoop.GetCurrentPlayerLoop();
	        for (var i = 0; i < loop.subSystemList.Length; i++)
	        {
	            var subSystem = loop.subSystemList[i];
	            if (subSystem.type == typeof(UnityEngine.PlayerLoop.FixedUpdate))
	            {
		            // Append FixedUpdate to subsytems
		            var newSubSystems = new PlayerLoopSystem[subSystem.subSystemList.Length + 1];
		            for (var j = 0; j < subSystem.subSystemList.Length; j++)
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
		            var newSubSystems = new PlayerLoopSystem[subSystem.subSystemList.Length + 1];
		            for (var j = 0; j < subSystem.subSystemList.Length; j++)
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
			var methodInfo = UnityEventBase.GetValidMethodInfo(component.GetType(), methodName, new[] { typeof(T) });
			m_Callback = (Callback)methodInfo?.CreateDelegate(typeof(Callback), component);
		}
	}
	
	static Dictionary<Rigidbody, RigidBodyHandle> rigidbodyToHandle = new ();
	static Dictionary<Collider, ColliderHandle> colliderToHandle = new ();
	static Dictionary<ColliderHandle, Collider> handleToCollider = new ();
	static Dictionary<Collider, RigidBodyHandle> fixedRigidbodies = new ();
	static Dictionary<Collider, EventsForCollider> physicsEvents = new ();
	static Dictionary<MonoBehaviour, EventsForMonoBehaviour> monoBehaviourEvents = new ();
	static HashSet<(Collider, Collider)> activeTriggerPairs = new ();
	
	public static void AddForceWithMode(Rigidbody rigidbody, Vector3 force, ForceMode mode)
	{
		// Debug.Log($"Adding force {force} to rigidbody {rigidbody}");
		var handle = rigidbodyToHandle[rigidbody];
		RapierBindings.add_force(handle, force.x, force.y, force.z, mode);
	}
	
	public static void AddForce(Rigidbody rigidbody, Vector3 force)
	{
		// Debug.Log($"Adding force {force} to rigidbody {rigidbody}");
		var handle = rigidbodyToHandle[rigidbody];
		RapierBindings.add_force(handle, force.x, force.y, force.z, ForceMode.Force);
	}
	
	public static void Initialization()
	{
		if (!Application.isPlaying)
			return;
		
		foreach (var collider in Object.FindObjectsByType<Collider>(FindObjectsSortMode.None))
		{
			if (!colliderToHandle.ContainsKey(collider))
			{
				var potentialRigidbody = collider.GetComponent<Rigidbody>();
				var transformScale = collider.transform.localScale;
				
				switch (collider)
				{
					case BoxCollider boxCollider:
					{
						var newColliderHandle = RapierBindings.add_cuboid_collider(transformScale.x * boxCollider.size.x / 2, transformScale.y * boxCollider.size.y / 2, transformScale.z * boxCollider.size.z / 2, potentialRigidbody == null ? 0 : potentialRigidbody.mass, boxCollider.isTrigger);
						colliderToHandle[collider] = newColliderHandle;
						handleToCollider[newColliderHandle] = collider;
						Debug.Log($"Added collider {collider} with handle {colliderToHandle[collider]}");
						break;
					}
					case SphereCollider sphereCollider:
					{
						var newColliderHandle = RapierBindings.add_sphere_collider(transformScale.x * sphereCollider.radius, potentialRigidbody == null ? 0 : potentialRigidbody.mass);
						colliderToHandle[collider] = newColliderHandle;
						handleToCollider[newColliderHandle] = collider;
						Debug.Log($"Added collider {collider} with handle {colliderToHandle[collider]}");
						break;
					}
				}


				var eventsForCollider = new EventsForCollider();
				
				eventsForCollider.onTriggerEnter = new List<BasicEvent<Collider>>();
				eventsForCollider.onTriggerExit = new List<BasicEvent<Collider>>();
				eventsForCollider.onTriggerStay = new List<BasicEvent<Collider>>();
				
				eventsForCollider.onCollisionEnter = new List<BasicEvent<Collision>>();
				eventsForCollider.onCollisionExit = new List<BasicEvent<Collision>>();
				eventsForCollider.onCollisionStay = new List<BasicEvent<Collision>>();
				
				var components = collider.gameObject.GetComponents<MonoBehaviour>();
				foreach (var component in components)
				{
					if (!monoBehaviourEvents.TryGetValue(component, out var eventsForMonoBehaviour))
					{
						eventsForMonoBehaviour.onTriggerEnter = new BasicEvent<Collider>(component, "OnTriggerEnter");
						eventsForMonoBehaviour.onTriggerExit = new BasicEvent<Collider>(component, "OnTriggerExit");
						eventsForMonoBehaviour.onTriggerStay = new BasicEvent<Collider>(component, "OnTriggerStay");
					
						eventsForMonoBehaviour.onCollisionEnter = new BasicEvent<Collision>(component, "OnCollisionEnter");
						eventsForMonoBehaviour.onCollisionExit = new BasicEvent<Collision>(component, "OnCollisionExit");
						eventsForMonoBehaviour.onCollisionStay = new BasicEvent<Collision>(component, "OnCollisionStay");
					
						monoBehaviourEvents[component] = eventsForMonoBehaviour;
					}
					
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

				if (potentialRigidbody == null && colliderToHandle.TryGetValue(collider, out var colliderHandle))
				{
					// Add Fixed RigidBody
					fixedRigidbodies[collider] = RapierBindings.add_rigid_body(colliderHandle, RigidBodyType.Fixed, collider.transform.position.x, collider.transform.position.y, collider.transform.position.z);
				}
			}
		}
		
		foreach (var rigidbody in Object.FindObjectsByType<Rigidbody>(FindObjectsSortMode.None))
		{
			if (!rigidbodyToHandle.ContainsKey(rigidbody))
			{
				var colliders = rigidbody.GetComponents<Collider>();
				Assert.AreEqual(colliders.Length, 1, "Rigidbody must have exactly one collider for the moment");
				var colliderHandle = colliderToHandle[colliders[0]];
				var trs = rigidbody.transform;
				rigidbodyToHandle[rigidbody] = RapierBindings.add_rigid_body(colliderHandle, RigidBodyType.Dynamic, trs.position.x, trs.position.y, trs.position.z);
				Debug.Log($"Added rigidbody {rigidbody} with handle {rigidbodyToHandle[rigidbody]} and collider handle {colliderHandle} which is {colliders[0]}");
			}
		}
		
	}
	
	public static void FixedUpdate()
	{
		if (!Application.isPlaying)
			return;

		unsafe
		{
			// var event_count = new NativeReference<int>(0, Allocator.Temp);
			var events = (RawArray<CollisionEvent>*)RapierBindings.solve();
			for (int i = 0; i < events->length; i++)
			{
				var @event = (*events)[i];
				var collider1 = handleToCollider[@event.collider1];
				var collider2 = handleToCollider[@event.collider2];
				var eventsForCollider1 = physicsEvents[collider1];
				var eventsForCollider2 = physicsEvents[collider2];
				if (@event.is_started)
				{
					if (collider1.isTrigger || collider2.isTrigger)
					{
						foreach (var enter in eventsForCollider1.onTriggerEnter)
							enter.Invoke(collider2);
						foreach (var enter in eventsForCollider2.onTriggerEnter)
							enter.Invoke(collider1);
					}
					
					activeTriggerPairs.Add((collider1, collider2));
				}
				else
				{
					if (collider1.isTrigger || collider2.isTrigger)
					{
						foreach (var exit in eventsForCollider1.onTriggerExit)
							exit.Invoke(collider2);
						foreach (var exit in eventsForCollider2.onTriggerExit)
							exit.Invoke(collider1);
					}
					activeTriggerPairs.Remove((collider1, collider2));
				}
			}

			foreach (var activeTriggerPair in activeTriggerPairs)
			{
				var (collider1, collider2) = activeTriggerPair;
				var eventsForCollider1 = physicsEvents[collider1];
				var eventsForCollider2 = physicsEvents[collider2];
				if (collider1.isTrigger || collider2.isTrigger)
				{
					foreach (var stay in eventsForCollider1.onTriggerStay)
						stay.Invoke(collider2);
					foreach (var stay in eventsForCollider2.onTriggerStay)
						stay.Invoke(collider1);
				}
			}
			RapierBindings.free_collision_events((IntPtr)events);
		}
		
		foreach (var rigidbody in Object.FindObjectsByType<Rigidbody>(FindObjectsSortMode.None))
		{
			Debug.Log($"Velocity: {rigidbody.linearVelocity}");
			var handle = rigidbodyToHandle[rigidbody];
			var position = RapierBindings.get_position(handle);
			rigidbody.transform.SetPositionAndRotation(position.position, position.rotation);
			
			// Debug.Log($"Updated rigidbody {rigidbody} with handle {handle} to position {position}");
		}
		

		foreach (var (collider, rbhandle) in fixedRigidbodies)
		{
			// var eventsForCollider = physicsEvents[GameObject.Find("Sphere").GetComponent<Collider>()];
			// foreach (var enter in eventsForCollider.onTriggerEnter)
			// 	enter.Invoke(collider);
			
			//var position = RapierBindings.get_position(rbhandle);
			//Debug.Log($"Updated fixed rigidbody {collider} with handle {rbhandle} to position {position}");
			
			//collider.transform.position = new Vector3(position.position.x, position.position.y, position.position.z);
			//collider.transform.rotation = new Quaternion(position.rotation.x, position.rotation.y, position.rotation.z, position.rotation.w);
		}
	}
}