
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using Mono.Cecil;
using Mono.Cecil.Cil;
using Unity.CompilationPipeline.Common.ILPostProcessing;
using Unity.Rapier4Unity.CodeGen;

public class Patch : ILPostProcessor {
    public static void OutputDebugString(string message)
    {
        File.AppendAllText("./Temp/rapier4unity.log", message + "\n");
    }

    public override ILPostProcessor GetInstance() {
        return new Patch();
    }

    public override bool WillProcess(ICompiledAssembly compiledAssembly) {
        string name = compiledAssembly.Name;
        if (name.StartsWith("Unity.") || name.StartsWith("UnityEngine.") || name.StartsWith("UnityEditor."))
            return false;

        //if (!compiledAssembly.References.Any(r => r.EndsWith("Cube.Replication.dll")))
            //return false; // No reference to Replication
        // if (compiledAssembly.Name != "Assembly-CSharp")
        //     return false;

        OutputDebugString($"{compiledAssembly.Name}: WillProcess");
        return true;
    }

    public override ILPostProcessResult Process(ICompiledAssembly compiledAssembly) {
        OutputDebugString($"{compiledAssembly.Name}: Start patching...");

        var msgs = new System.Collections.Generic.List<Unity.CompilationPipeline.Common.Diagnostics.DiagnosticMessage>();

        try {
            // using (var stream = new MemoryStream(compiledAssembly.InMemoryAssembly.PeData)) {
            //     var resolver = new DefaultAssemblyResolver();
            //     foreach (var path in compiledAssembly.References) {
            //         var dir = Path.GetDirectoryName(path);
            //         if (resolver.GetSearchDirectories().Contains(dir))
            //             continue;
            //
            //         OutputDebugString($"{compiledAssembly.Name}: Search in {dir}");
            //         resolver.AddSearchDirectory(dir);
            //     }
            //
            //     var readerParameters = new ReaderParameters() {
            //         AssemblyResolver = resolver,
            //     };

 //               using (var assembly = AssemblyDefinition.ReadAssembly(stream, readerParameters)) {
                    var assembly = compiledAssembly.GetAssemblyDefinition();
                    var rpcProcessor = new PhysicsPostProcessor(assembly.MainModule);
                    var anythingChanged = rpcProcessor.Process(assembly.MainModule);
                    if (!anythingChanged) {
                        OutputDebugString($"{compiledAssembly.Name}: NOTHING CHANGED");
                        return new ILPostProcessResult(compiledAssembly.InMemoryAssembly);
                    }
                    
                    var pe = new MemoryStream();
                    var pdb = new MemoryStream();
                    var writerParameters = new WriterParameters
                    {
                        SymbolWriterProvider = new PortablePdbWriterProvider(),
                        SymbolStream = pdb,
                        WriteSymbols = true
                    };

                    assembly.Write(pe, writerParameters);
                    return new ILPostProcessResult(new InMemoryAssembly(pe.ToArray(), pdb.ToArray()), msgs);
                    
                    // using (var outStream = new MemoryStream()) {
                    //     assembly.Write(outStream);
                    //
                    //     OutputDebugString($"{compiledAssembly.Name}: SUCCESS");
                    //
                    //     return new ILPostProcessResult(new InMemoryAssembly(outStream.ToArray(), compiledAssembly.InMemoryAssembly.PdbData), msgs);
                    // }
                // }
        //     }
        } catch (System.Exception e) {
            var msg = new Unity.CompilationPipeline.Common.Diagnostics.DiagnosticMessage();
            msg.DiagnosticType = Unity.CompilationPipeline.Common.Diagnostics.DiagnosticType.Error;
            msg.MessageData = e.Message;
            msgs.Add(msg);
        
            OutputDebugString($"{compiledAssembly.Name}: FAILED {e.Message}");
            return new ILPostProcessResult(null, msgs);
        }
    }
}

public class PhysicsPostProcessor
{
    ModuleDefinition m_AssemblyMainModule;
    public PhysicsPostProcessor(ModuleDefinition assemblyMainModule)
    {
        m_AssemblyMainModule = assemblyMainModule;
    }

    public bool Process(ModuleDefinition assemblyMainModule)
    {
        bool anythingChanged = false;

        foreach (var type in assemblyMainModule.Types) {
            foreach (var method in type.Methods) {
                if (!method.HasBody)
                    continue;
                
                var instructions = method.Body.Instructions;
                for (int i = 0; i < instructions.Count; i++) {
                    var instruction = instructions[i];
                    
                    if (instruction.OpCode == Mono.Cecil.Cil.OpCodes.Callvirt) {
                        if (instruction.Operand is not MethodReference methodReference)
                            continue;
                        if (methodReference.DeclaringType.FullName == "UnityEngine.Rigidbody" && methodReference.Name == "AddForce") {
                            if (methodReference.Parameters.Count == 1)
                            {
                                var addForce = typeof(RapierLoop).GetMethod("AddForce", System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic | BindingFlags.Public);
                                var newMethodReference = m_AssemblyMainModule.ImportReference(addForce);
                                Patch.OutputDebugString($"Method: {GetMethodSignature(methodReference)} -> {GetMethodSignature(newMethodReference)}");
                                instruction.Operand = newMethodReference;
                            } else if (methodReference.Parameters.Count == 2)
                            {
                                var addForce = typeof(RapierLoop).GetMethod("AddForceWithMode", System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic | BindingFlags.Public);
                                var newMethodReference = m_AssemblyMainModule.ImportReference(addForce);
                                Patch.OutputDebugString($"Method: {GetMethodSignature(methodReference)} -> {GetMethodSignature(newMethodReference)}");
                                instruction.Operand = newMethodReference;
                            }
                            anythingChanged = true;
                        }
                    } else if (instruction.OpCode == Mono.Cecil.Cil.OpCodes.Call) {
                        var methodReference = instruction.Operand as MethodReference;
                        if (methodReference == null)
                            continue;
                        // Patch.OutputDebugString($"Method: {methodReference.DeclaringType.FullName}.{methodReference.Name}");
                
                        // if (methodReference.DeclaringType.FullName == "SpacetimeDB.Types.PlayerId" && methodReference.Name == "Find") {
                        //     var newMethodReference = m_AssemblyMainModule.ImportReference(typeof(PrefabManager).GetMethod("SpawnPlayer"));
                        //     instruction.Operand = newMethodReference;
                        //     anythingChanged = true;
                        // }
                    } else if (instruction.OpCode == Mono.Cecil.Cil.OpCodes.Calli) {
                        //Patch.OutputDebugString($"Calli: {instruction.Operand}");
                    }
                }
            }
        }

        return anythingChanged;
    }
    
    public static string GetMethodSignature(MethodReference methodReference)
    {
        return $"{methodReference.DeclaringType.FullName}.{methodReference.Name}({string.Join(", ", methodReference.Parameters.Select(p => p.ParameterType.FullName))})";
    }
}