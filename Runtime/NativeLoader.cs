using System;
using System.Runtime.InteropServices;

namespace Packages.rapier4unity.Runtime
{
    public static class NativeLoader
    {
        private const string LibSystem = "/usr/lib/libSystem.dylib";

        // Flags for dlopen
        [Flags]
        public enum LoadMode : int
        {
            Lazy = 0x1,		// Only resolve symbols as needed
            Now = 0x2,		// Resolve all symbols immediately
            Global = 0x8	// Symbols in this library are available to libraries loaded after this one
        }

        [DllImport(LibSystem)]
        public static extern IntPtr dlopen(string path, LoadMode mode);

        [DllImport(LibSystem)]
        public static extern IntPtr dlsym(IntPtr handle, string symbol);

        [DllImport(LibSystem)]
        public static extern int dlclose(IntPtr handle);

        [DllImport(LibSystem)]
        public static extern IntPtr dlerror(); // Get error message
    }
    
    public static class UnityRapierBridge
    {
#if UNITY_STANDALONE_OSX
        private const string DllName = "libunitybridge.dylib";
#else
		private const string DllName = "unitybridge";
#endif

        private const CallingConvention Convention = CallingConvention.Cdecl;
        
        [DllImport(DllName, CallingConvention = Convention)]
        public static extern IntPtr GetDefaultUnityLogger();
    }
}
