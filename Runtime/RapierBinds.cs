//#define DISABLE_DYNAMIC_RAPIER_LOAD

using System;
using System.IO;
using System.Runtime.InteropServices;
using Packages.rapier4unity.Runtime;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Burst;
using UnityEngine;
using Unity.Mathematics;

#if UNITY_EDITOR && !DISABLE_DYNAMIC_RAPIER_LOAD
[UnityEditor.InitializeOnLoad]
#endif

[BurstCompile]
internal static unsafe class RapierBindings
{
#if UNITY_STANDALONE_OSX
	private const string DllName = "librapier_c_bind.dylib";
	const string k_DLLPath = "Packages/rapier4unity/build_bin/librapier_c_bind.dylib";
#else
	private const string DllName = "rapier_c_bind";
	const string k_DLLPath = "Packages/rapier4unity/build_bin/rapier_c_bind.dll";
#endif
	private const CallingConvention Convention = CallingConvention.Cdecl;
#if UNITY_EDITOR && !DISABLE_DYNAMIC_RAPIER_LOAD
	public static void Init(FunctionsToCallFromRust* funcs) => ((delegate* unmanaged[Cdecl]<FunctionsToCallFromRust*, void>) data.Data.init)(funcs);
	public static void HelloWorld() => ((delegate* unmanaged[Cdecl]<void>) data.Data.helloWorld)();
	public static void Teardown() => ((delegate* unmanaged[Cdecl]<void>) data.Data.teardown)();
	public static RawArray<CollisionEvent>* Solve() => ((delegate* unmanaged[Cdecl]<RawArray<CollisionEvent>*>) data.Data.solve)();
	public static void FreeCollisionEvents(RawArray<CollisionEvent>* ptr) => ((delegate* unmanaged[Cdecl]<RawArray<CollisionEvent>*, void>) data.Data.freeCollisionEvents)(ptr);
	public static void SetGravity(float x, float y, float z) => ((delegate* unmanaged[Cdecl]<float, float, float, void>) data.Data.setGravity)(x, y, z);
	public static void SetTimeStep(float dt) => ((delegate* unmanaged[Cdecl]<float, void>) data.Data.setTimeStep)(dt);
	public static ColliderHandle AddCuboidCollider(float halfExtentsX, float halfExtentsY, float halfExtentsZ, float mass, bool isSensor) => ((delegate* unmanaged[Cdecl]<float, float, float, float, bool, ColliderHandle>) data.Data.addCuboidCollider)(halfExtentsX, halfExtentsY, halfExtentsZ, mass, isSensor);
	public static ColliderHandle AddSphereCollider(float radius, float mass, bool isSensor) => ((delegate* unmanaged[Cdecl]<float, float, bool, ColliderHandle>) data.Data.addSphereCollider)(radius, mass, isSensor);
	public static ColliderHandle AddCapsuleCollider(float halfHeight, float radius, float mass, bool isSensor) => ((delegate* unmanaged[Cdecl]<float, float, float, bool, ColliderHandle>) data.Data.addCapsuleCollider)(halfHeight, radius, mass, isSensor);
	public static ColliderHandle AddMeshCollider(float* verticesPtr, UIntPtr verticesCount, uint* indicesPtr, UIntPtr indicesCount, float mass, bool isSensor) => ((delegate* unmanaged[Cdecl]<float*, UIntPtr, uint*, UIntPtr, float, bool, ColliderHandle>) data.Data.addMeshCollider)(verticesPtr, verticesCount, indicesPtr, indicesCount, mass, isSensor);
	public static ColliderHandle AddConvexMeshCollider(float* verticesPtr, UIntPtr verticesCount, float mass, bool isSensor) => ((delegate* unmanaged[Cdecl]<float*, UIntPtr, float, bool, ColliderHandle>) data.Data.addConvexMeshCollider)(verticesPtr, verticesCount, mass, isSensor);
	public static RigidBodyHandle AddRigidBody(ColliderHandle collider, RigidBodyType rbType, float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW) => ((delegate* unmanaged[Cdecl]<ColliderHandle, RigidBodyType, float, float, float, float, float, float, float, RigidBodyHandle>) data.Data.addRigidBody)(collider, rbType, positionX, positionY, positionZ, rotationX, rotationY, rotationZ, rotationW);
	public static void RemoveRigidBody(RigidBodyHandle rbHandle) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, void>) data.Data.removeRigidBody)(rbHandle);
	public static void SetRigidBodyType(RigidBodyHandle rbHandle, RigidBodyType rbType) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, RigidBodyType, void>) data.Data.setRigidBodyType)(rbHandle, rbType);
	public static void SetRigidBodyConstraints(RigidBodyHandle rbHandle, uint constraints) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, uint, void>) data.Data.setRigidBodyConstraints)(rbHandle, constraints);
	public static ImpulseJointHandle AddFixedJoint(RigidBodyHandle rb1Handle, RigidBodyHandle rb2Handle, float localFrame1X, float localFrame1Y, float localFrame1Z, float localFrame2X, float localFrame2Y, float localFrame2Z, bool selfCollision) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, RigidBodyHandle, float, float, float, float, float, float, bool, ImpulseJointHandle>) data.Data.addFixedJoint)(rb1Handle, rb2Handle, localFrame1X, localFrame1Y, localFrame1Z, localFrame2X, localFrame2Y, localFrame2Z, selfCollision);
	public static ImpulseJointHandle AddSphericalJoint(RigidBodyHandle rb1Handle, RigidBodyHandle rb2Handle, float localFrame1X, float localFrame1Y, float localFrame1Z, float localFrame2X, float localFrame2Y, float localFrame2Z, bool selfCollision) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, RigidBodyHandle, float, float, float, float, float, float, bool, ImpulseJointHandle>) data.Data.addSphericalJoint)(rb1Handle, rb2Handle, localFrame1X, localFrame1Y, localFrame1Z, localFrame2X, localFrame2Y, localFrame2Z, selfCollision);
	public static ImpulseJointHandle AddRevoluteJoint(RigidBodyHandle rb1Handle, RigidBodyHandle rb2Handle, float axisX, float axisY, float axisZ, float localFrame1X, float localFrame1Y, float localFrame1Z, float localFrame2X, float localFrame2Y, float localFrame2Z, bool selfCollision) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, RigidBodyHandle, float, float, float, float, float, float, float, float, float, bool, ImpulseJointHandle>) data.Data.addRevoluteJoint)(rb1Handle, rb2Handle, axisX, axisY, axisZ, localFrame1X, localFrame1Y, localFrame1Z, localFrame2X, localFrame2Y, localFrame2Z, selfCollision);
	public static ImpulseJointHandle AddPrismaticJoint(RigidBodyHandle rb1Handle, RigidBodyHandle rb2Handle, float axisX, float axisY, float axisZ, float localFrame1X, float localFrame1Y, float localFrame1Z, float localFrame2X, float localFrame2Y, float localFrame2Z, float limitMin, float limitMax, bool selfCollision) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, RigidBodyHandle, float, float, float, float, float, float, float, float, float, float, float, bool, ImpulseJointHandle>) data.Data.addPrismaticJoint)(rb1Handle, rb2Handle, axisX, axisY, axisZ, localFrame1X, localFrame1Y, localFrame1Z, localFrame2X, localFrame2Y, localFrame2Z, limitMin, limitMax, selfCollision);
	public static void RemoveJoint(ImpulseJointHandle handle) => ((delegate* unmanaged[Cdecl]<ImpulseJointHandle, void>) data.Data.removeJoint)(handle);
	public static RapierTransform GetTransform(RigidBodyHandle rbHandle) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, RapierTransform>) data.Data.getTransform)(rbHandle);
	public static void SetTransformPosition(RigidBodyHandle rbHandle, float positionX, float positionY, float positionZ) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, float, float, float, void>) data.Data.setTransformPosition)(rbHandle, positionX, positionY, positionZ);
	public static void SetTransformRotation(RigidBodyHandle rbHandle, float rotationX, float rotationY, float rotationZ, float rotationW) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, float, float, float, float, void>) data.Data.setTransformRotation)(rbHandle, rotationX, rotationY, rotationZ, rotationW);
	public static void SetTransform(RigidBodyHandle rbHandle, float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, float, float, float, float, float, float, float, void>) data.Data.setTransform)(rbHandle, positionX, positionY, positionZ, rotationX, rotationY, rotationZ, rotationW);
	public static void SetLinearVelocity(RigidBodyHandle rbHandle, float velocityX, float velocityY, float velocityZ) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, float, float, float, void>) data.Data.setLinearVelocity)(rbHandle, velocityX, velocityY, velocityZ);
	public static void SetAngularVelocity(RigidBodyHandle rbHandle, float velocityX, float velocityY, float velocityZ) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, float, float, float, void>) data.Data.setAngularVelocity)(rbHandle, velocityX, velocityY, velocityZ);
	public static float3 GetLinearVelocity(RigidBodyHandle rbHandle) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, float3>) data.Data.getLinearVelocity)(rbHandle);
	public static float3 GetAngularVelocity(RigidBodyHandle rbHandle) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, float3>) data.Data.getAngularVelocity)(rbHandle);
	public static void EnableCCD(RigidBodyHandle rbHandle, bool enabled) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, bool, void>) data.Data.enableCCD)(rbHandle, enabled);
	public static void AddForce(RigidBodyHandle rbHandle, float forceX, float forceY, float forceZ, ForceMode mode) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, float, float, float, ForceMode, void>) data.Data.addForce)(rbHandle, forceX, forceY, forceZ, mode);
	public static void AddTorque(RigidBodyHandle rbHandle, float torqueX, float torqueY, float torqueZ, ForceMode mode) => ((delegate* unmanaged[Cdecl]<RigidBodyHandle, float, float, float, ForceMode, void>) data.Data.addTorque)(rbHandle, torqueX, torqueY, torqueZ, mode);
	public static bool CastRay(float fromX, float fromY, float fromZ, float dirX, float dirY, float dirZ, RapierRaycastHit* outHit) => ((delegate* unmanaged[Cdecl]<float, float, float, float, float, float, RapierRaycastHit*, bool>) data.Data.castRay)(fromX, fromY, fromZ, dirX, dirY, dirZ, outHit);
#else
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="init")]
	public static extern unsafe void Init(FunctionsToCallFromRust* funcs);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="hello_world")]
	public static extern unsafe void HelloWorld();
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="teardown")]
	public static extern unsafe void Teardown();
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="solve")]
	public static extern unsafe RawArray<CollisionEvent>* Solve();
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="free_collision_events")]
	public static extern unsafe void FreeCollisionEvents(RawArray<CollisionEvent>* ptr);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="set_gravity")]
	public static extern unsafe void SetGravity(float x, float y, float z);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="set_time_step")]
	public static extern unsafe void SetTimeStep(float dt);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_cuboid_collider")]
	public static extern unsafe ColliderHandle AddCuboidCollider(float halfExtentsX, float halfExtentsY, float halfExtentsZ, float mass, bool isSensor);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_sphere_collider")]
	public static extern unsafe ColliderHandle AddSphereCollider(float radius, float mass, bool isSensor);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_capsule_collider")]
	public static extern unsafe ColliderHandle AddCapsuleCollider(float halfHeight, float radius, float mass, bool isSensor);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_mesh_collider")]
	public static extern unsafe ColliderHandle AddMeshCollider(float* verticesPtr, UIntPtr verticesCount, uint* indicesPtr, UIntPtr indicesCount, float mass, bool isSensor);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_convex_mesh_collider")]
	public static extern unsafe ColliderHandle AddConvexMeshCollider(float* verticesPtr, UIntPtr verticesCount, float mass, bool isSensor);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_rigid_body")]
	public static extern unsafe RigidBodyHandle AddRigidBody(ColliderHandle collider, RigidBodyType rbType, float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="remove_rigid_body")]
	public static extern unsafe void RemoveRigidBody(RigidBodyHandle rbHandle);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="set_rigid_body_type")]
	public static extern unsafe void SetRigidBodyType(RigidBodyHandle rbHandle, RigidBodyType rbType);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="set_rigid_body_constraints")]
	public static extern unsafe void SetRigidBodyConstraints(RigidBodyHandle rbHandle, uint constraints);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_fixed_joint")]
	public static extern unsafe ImpulseJointHandle AddFixedJoint(RigidBodyHandle rb1Handle, RigidBodyHandle rb2Handle, float localFrame1X, float localFrame1Y, float localFrame1Z, float localFrame2X, float localFrame2Y, float localFrame2Z, bool selfCollision);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_spherical_joint")]
	public static extern unsafe ImpulseJointHandle AddSphericalJoint(RigidBodyHandle rb1Handle, RigidBodyHandle rb2Handle, float localFrame1X, float localFrame1Y, float localFrame1Z, float localFrame2X, float localFrame2Y, float localFrame2Z, bool selfCollision);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_revolute_joint")]
	public static extern unsafe ImpulseJointHandle AddRevoluteJoint(RigidBodyHandle rb1Handle, RigidBodyHandle rb2Handle, float axisX, float axisY, float axisZ, float localFrame1X, float localFrame1Y, float localFrame1Z, float localFrame2X, float localFrame2Y, float localFrame2Z, bool selfCollision);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_prismatic_joint")]
	public static extern unsafe ImpulseJointHandle AddPrismaticJoint(RigidBodyHandle rb1Handle, RigidBodyHandle rb2Handle, float axisX, float axisY, float axisZ, float localFrame1X, float localFrame1Y, float localFrame1Z, float localFrame2X, float localFrame2Y, float localFrame2Z, float limitMin, float limitMax, bool selfCollision);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="remove_joint")]
	public static extern unsafe void RemoveJoint(ImpulseJointHandle handle);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="get_transform")]
	public static extern unsafe RapierTransform GetTransform(RigidBodyHandle rbHandle);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="set_transform_position")]
	public static extern unsafe void SetTransformPosition(RigidBodyHandle rbHandle, float positionX, float positionY, float positionZ);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="set_transform_rotation")]
	public static extern unsafe void SetTransformRotation(RigidBodyHandle rbHandle, float rotationX, float rotationY, float rotationZ, float rotationW);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="set_transform")]
	public static extern unsafe void SetTransform(RigidBodyHandle rbHandle, float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="set_linear_velocity")]
	public static extern unsafe void SetLinearVelocity(RigidBodyHandle rbHandle, float velocityX, float velocityY, float velocityZ);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="set_angular_velocity")]
	public static extern unsafe void SetAngularVelocity(RigidBodyHandle rbHandle, float velocityX, float velocityY, float velocityZ);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="get_linear_velocity")]
	public static extern unsafe float3 GetLinearVelocity(RigidBodyHandle rbHandle);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="get_angular_velocity")]
	public static extern unsafe float3 GetAngularVelocity(RigidBodyHandle rbHandle);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="enable_CCD")]
	public static extern unsafe void EnableCCD(RigidBodyHandle rbHandle, bool enabled);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_force")]
	public static extern unsafe void AddForce(RigidBodyHandle rbHandle, float forceX, float forceY, float forceZ, ForceMode mode);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="add_torque")]
	public static extern unsafe void AddTorque(RigidBodyHandle rbHandle, float torqueX, float torqueY, float torqueZ, ForceMode mode);
	[DllImport(DllName, CallingConvention = Convention, EntryPoint="cast_ray")]
	public static extern unsafe bool CastRay(float fromX, float fromY, float fromZ, float dirX, float dirY, float dirZ, RapierRaycastHit* outHit);
#endif

#if UNITY_EDITOR && !DISABLE_DYNAMIC_RAPIER_LOAD
    // C# -> Rust
    [StructLayout(LayoutKind.Sequential, Size=sizeof(ulong) * 512)]
    struct UnmanagedData
    {
        IntPtr loaded_lib;
        public void load_calls()
        {
            if (loaded_lib==IntPtr.Zero)
            {
                loaded_lib = NativeLoader.LoadLibrary(Path.GetFullPath(k_DLLPath));
                if (loaded_lib == IntPtr.Zero)
                {
                    var errorMsg = NativeLoader.GetLastErrorString();
                    Console.WriteLine($"Failed to load library: {errorMsg}");
                    return;
                }
            }

            // Load function pointers
            init = NativeLoader.GetFunction(loaded_lib, "init");
			helloWorld = NativeLoader.GetFunction(loaded_lib, "hello_world");
			teardown = NativeLoader.GetFunction(loaded_lib, "teardown");
			solve = NativeLoader.GetFunction(loaded_lib, "solve");
			freeCollisionEvents = NativeLoader.GetFunction(loaded_lib, "free_collision_events");
			setGravity = NativeLoader.GetFunction(loaded_lib, "set_gravity");
			setTimeStep = NativeLoader.GetFunction(loaded_lib, "set_time_step");
			addCuboidCollider = NativeLoader.GetFunction(loaded_lib, "add_cuboid_collider");
			addSphereCollider = NativeLoader.GetFunction(loaded_lib, "add_sphere_collider");
			addCapsuleCollider = NativeLoader.GetFunction(loaded_lib, "add_capsule_collider");
			addMeshCollider = NativeLoader.GetFunction(loaded_lib, "add_mesh_collider");
			addConvexMeshCollider = NativeLoader.GetFunction(loaded_lib, "add_convex_mesh_collider");
			addRigidBody = NativeLoader.GetFunction(loaded_lib, "add_rigid_body");
			removeRigidBody = NativeLoader.GetFunction(loaded_lib, "remove_rigid_body");
			setRigidBodyType = NativeLoader.GetFunction(loaded_lib, "set_rigid_body_type");
			setRigidBodyConstraints = NativeLoader.GetFunction(loaded_lib, "set_rigid_body_constraints");
			addFixedJoint = NativeLoader.GetFunction(loaded_lib, "add_fixed_joint");
			addSphericalJoint = NativeLoader.GetFunction(loaded_lib, "add_spherical_joint");
			addRevoluteJoint = NativeLoader.GetFunction(loaded_lib, "add_revolute_joint");
			addPrismaticJoint = NativeLoader.GetFunction(loaded_lib, "add_prismatic_joint");
			removeJoint = NativeLoader.GetFunction(loaded_lib, "remove_joint");
			getTransform = NativeLoader.GetFunction(loaded_lib, "get_transform");
			setTransformPosition = NativeLoader.GetFunction(loaded_lib, "set_transform_position");
			setTransformRotation = NativeLoader.GetFunction(loaded_lib, "set_transform_rotation");
			setTransform = NativeLoader.GetFunction(loaded_lib, "set_transform");
			setLinearVelocity = NativeLoader.GetFunction(loaded_lib, "set_linear_velocity");
			setAngularVelocity = NativeLoader.GetFunction(loaded_lib, "set_angular_velocity");
			getLinearVelocity = NativeLoader.GetFunction(loaded_lib, "get_linear_velocity");
			getAngularVelocity = NativeLoader.GetFunction(loaded_lib, "get_angular_velocity");
			enableCCD = NativeLoader.GetFunction(loaded_lib, "enable_CCD");
			addForce = NativeLoader.GetFunction(loaded_lib, "add_force");
			addTorque = NativeLoader.GetFunction(loaded_lib, "add_torque");
			castRay = NativeLoader.GetFunction(loaded_lib, "cast_ray");
        }

        // Raw function pointers
        public IntPtr init;
		public IntPtr helloWorld;
		public IntPtr teardown;
		public IntPtr solve;
		public IntPtr freeCollisionEvents;
		public IntPtr setGravity;
		public IntPtr setTimeStep;
		public IntPtr addCuboidCollider;
		public IntPtr addSphereCollider;
		public IntPtr addCapsuleCollider;
		public IntPtr addMeshCollider;
		public IntPtr addConvexMeshCollider;
		public IntPtr addRigidBody;
		public IntPtr removeRigidBody;
		public IntPtr setRigidBodyType;
		public IntPtr setRigidBodyConstraints;
		public IntPtr addFixedJoint;
		public IntPtr addSphericalJoint;
		public IntPtr addRevoluteJoint;
		public IntPtr addPrismaticJoint;
		public IntPtr removeJoint;
		public IntPtr getTransform;
		public IntPtr setTransformPosition;
		public IntPtr setTransformRotation;
		public IntPtr setTransform;
		public IntPtr setLinearVelocity;
		public IntPtr setAngularVelocity;
		public IntPtr getLinearVelocity;
		public IntPtr getAngularVelocity;
		public IntPtr enableCCD;
		public IntPtr addForce;
		public IntPtr addTorque;
		public IntPtr castRay;

        // Rust -> C# data
        public FunctionsToCallFromRust functionsToCallFromRust;

        public void unload_calls()
        {
            if (loaded_lib != IntPtr.Zero)
                NativeLoader.FreeLibrary(loaded_lib);
            loaded_lib = IntPtr.Zero;
            Debug.Log($"RapierBindingCalls Unloaded");
        }
        public bool IsLoaded => loaded_lib != IntPtr.Zero;
    }
    public static bool IsAvailable => data.Data.IsLoaded;
    public static void LoadCalls()
    {
        data.Data.load_calls();
        init_rapier4unity();
        Debug.Log($"RapierBindingCalls Loaded");
    }

    [BurstCompile]
    static void init_rapier4unity(){
        data.Data.functionsToCallFromRust.Init();
        Init((FunctionsToCallFromRust*)UnsafeUtility.AddressOf(ref data.Data.functionsToCallFromRust));
    }

    public static void UnloadCalls() => data.Data.unload_calls();

    struct SpecialKey {}
    static readonly SharedStatic<UnmanagedData> data = SharedStatic<UnmanagedData>.GetOrCreate<SpecialKey>();
#else
    // No dynamic loading, empty functions
    public static bool IsAvailable => true;
    public static void LoadCalls(){}
    public static void UnloadCalls(){}

    // Called during InitializeOnLoad
	static RapierBindings()
	{
		var functionsToCallFromRust = new FunctionsToCallFromRust();
		functionsToCallFromRust.Init();
		Init(&functionsToCallFromRust);
	}
#endif

    // Rust -> C#
    [BurstCompile]
    [StructLayout(LayoutKind.Sequential)]
    public struct FunctionsToCallFromRust {
        public IntPtr unityLogPtr;

        public void Init()
        {
            unityLogPtr = UnityRapierBridge.GetDefaultUnityLogger();
        }
    }

}
