using System;
using System.Runtime.InteropServices;

namespace Packages.rapier4unity.Runtime
{
    public static class NativeLoader
    {
        
#if UNITY_STANDALONE_OSX
        private const string LibSystem = "/usr/lib/libSystem.dylib";

        // Flags for dlopen
        [Flags]
        public enum LoadMode : int
        {
            /// Only resolve symbols as needed
            Lazy = 0x1,		
            /// Resolve all symbols immediately
            Now = 0x2,		
            /// Symbols in this library are available to libraries loaded after this one
            Global = 0x8	
        }

        [DllImport(LibSystem)]
        static extern IntPtr dlopen(string path, LoadMode mode);

        [DllImport(LibSystem)]
        static extern IntPtr dlsym(IntPtr handle, string symbol);

        [DllImport(LibSystem)]
        static extern int dlclose(IntPtr handle);

        [DllImport(LibSystem)]
        static extern IntPtr dlerror(); // Get error message
#else
        private const string LibWindows = "kernel32.dll";
        
        [DllImport(LibWindows)]
        static extern IntPtr LoadLibraryA(string path);
        
        [DllImport(LibWindows)]
        static extern IntPtr GetProcAddress(IntPtr handle, string symbol);
        
        [DllImport(LibWindows, EntryPoint = "FreeLibrary")]
        static extern bool Win32FreeLibrary(IntPtr handle);
        
        [DllImport(LibWindows)]
        static extern IntPtr GetLastError();
#endif


        /// <summary>
        /// Load a library from the path (relative or absolute)
        /// DLL on Windows, dylib on OSX
        /// </summary>
        /// <param name="path">Path (can be relative to Unity project root or absolute) </param>
        /// <returns> Handle to Library </returns>
        public static IntPtr LoadLibrary(string path) =>
#if UNITY_STANDALONE_OSX && UNITY_EDITOR
            dlopen(path, LoadMode.Lazy);
#elif UNITY_EDITOR
            LoadLibraryA(path);
#else
            throw new PlatformNotSupportedException("Loading libraries is not supported on player");
#endif
        
        /// <summary>
        ///  Get a function pointer from the library
        /// </summary>
        /// <param name="handle"> Library Handle gotten from <see cref="LoadLibrary"/> </param>
        /// <param name="symbol"> Name of the Symbol to load from the library </param>
        /// <returns></returns>
        public static IntPtr GetFunction(IntPtr handle, string symbol) =>
#if UNITY_STANDALONE_OSX
            dlsym(handle, symbol);
#else
            GetProcAddress(handle, symbol);
#endif

        /// <summary>
        ///  Free the library handle
        /// </summary>
        /// <param name="handle"> The Library Handle gotten from <see cref="LoadLibrary"/> </param>
        public static void FreeLibrary(IntPtr handle) =>
#if UNITY_STANDALONE_OSX
            dlclose(handle);
#else
            Win32FreeLibrary(handle);
#endif
        
        /// <summary>
        ///   Get the last error message from the native library
        /// </summary>
        /// <returns> The Error Message </returns>
        public static string GetLastErrorString() =>
#if UNITY_STANDALONE_OSX
            Marshal.PtrToStringAnsi(dlerror());
#else 
            Marshal.PtrToStringAnsi(GetLastError());
#endif
        
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
