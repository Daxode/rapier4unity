using System.Runtime.InteropServices;
using Unity.Mathematics;
using UnityEngine;
using System;
using System.IO;
using Unity.Burst;
using UnityEditor;

namespace Packages.rapier4unity.Runtime
{
    
#if !UNITY_EDITOR_OSX
    // Ensure DLL is only loaded in playmode
    [InitializeOnLoad]
    static class odecs_initializer {
        static odecs_initializer() {
            EditorApplication.playModeStateChanged += state => {
                if (state == PlayModeStateChange.EnteredPlayMode)
                    odecs_calls.load_calls();
                if (state == PlayModeStateChange.ExitingPlayMode)
                    odecs_calls.unload_calls();
            };
        }
    }

    [BurstCompile]
    unsafe static class odecs_calls {

        // C# -> Odin
        struct UnmanagedData {
            IntPtr loaded_lib;
            public void load_calls() {
                if (loaded_lib==IntPtr.Zero)
                    loaded_lib = win32.LoadLibrary(Path.GetFullPath("Packages/rapier4unity/build_bin/librapier_c_bind.dylib")); // TODO: this won't work for builds and Unity Package Manager location
                init = win32.GetProcAddress(loaded_lib, "init");
                // Rotate =  win32.GetProcAddress(loaded_lib, "Rotate");
            }

            public void unload_calls() {
                if (loaded_lib != IntPtr.Zero)
                    win32.FreeLibrary(loaded_lib);
                loaded_lib = IntPtr.Zero;
            }

            public IntPtr init;
            public IntPtr Rotate;
            public FunctionsToCallFromOdin functionsToCallFromOdin;
            public bool IsLoaded => loaded_lib != IntPtr.Zero;
        }

        // public static delegate* unmanaged[Cdecl]<ref SystemState, ref EntityQuery, void*, void*, void> Rotate 
        //     => (delegate* unmanaged[Cdecl]<ref SystemState, ref EntityQuery, void*, void*, void>) data.Data.Rotate;

        public static bool IsAvailable => data.Data.IsLoaded;

        public static void load_calls() 
        {
            data.Data.load_calls();
            init_rapier4unity();
        }
        [BurstCompile]
        static void init_rapier4unity(){
            data.Data.functionsToCallFromOdin.Init();
            var init = (delegate* unmanaged[Cdecl]<ref FunctionsToCallFromOdin, void>) data.Data.init;
            init(ref data.Data.functionsToCallFromOdin);
        }
        public static void unload_calls() => data.Data.unload_calls();

        struct SpecialKey {}
        static readonly SharedStatic<UnmanagedData> data = SharedStatic<UnmanagedData>.GetOrCreate<SpecialKey>();

        // Odin -> C#
        [BurstCompile]
        struct FunctionsToCallFromOdin {
            // IntPtr odecsContext;

            public void Init() {
                // odecsContext = Daxode.UnityInterfaceBridge.OdecsUnityBridge.GetDefaultOdecsContext();
            }
        }
    }


    // Kernel loading functions
    static class win32 {
        [DllImport("kernel32")]
        public static extern IntPtr LoadLibrary(string dllToLoad);
        
        [DllImport("kernel32")]
        public static extern IntPtr GetProcAddress(IntPtr dllPtr, string functionName);
        
        [DllImport("kernel32")]
        public static extern bool FreeLibrary(IntPtr dllPtr);
    }
#endif
    
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
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern void init();
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern void teardown();
        
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern unsafe IntPtr solve();
        
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern unsafe void free_collision_events(IntPtr events);
        
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern void set_gravity(float x, float y, float z);
        
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern void set_time_step(float x);
        
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern ColliderHandle add_cuboid_collider(float half_extents_x, float half_extents_y, float half_extents_z, float mass, bool is_sensor);
        
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern ColliderHandle add_sphere_collider(float radius, float mass);
        
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern RigidBodyHandle add_rigid_body(ColliderHandle collider, RigidBodyType rigidBodyType, float position_x, float position_y, float position_z);
        
        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern RapierPosition get_position(RigidBodyHandle rigidBodyHandle);

        [DllImport("librapier_c_bind.dylib", CallingConvention = CallingConvention.Cdecl)]
        public static extern void add_force(RigidBodyHandle handle, float forceX, float forceY, float forceZ, ForceMode mode);
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
