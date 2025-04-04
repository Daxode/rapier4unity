using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using Mono.Cecil;
using Mono.Cecil.Cil;
using Unity.CompilationPipeline.Common.ILPostProcessing;
using Unity.Rapier4Unity.CodeGen;

public class Patch : ILPostProcessor
{
    [Conditional("DEBUG")]
    public static void OutputDebugString(string message)
    {
        //File.AppendAllTextAsync("./Temp/rapier4unity.log", message + "\n");
    }
    public override ILPostProcessor GetInstance() => new Patch();

    public override bool WillProcess(ICompiledAssembly compiledAssembly)
    {
        string name = compiledAssembly.Name;
        if (name.StartsWith("Unity.") || name.StartsWith("UnityEngine.") || name.StartsWith("UnityEditor."))
            return false;

        OutputDebugString($"{compiledAssembly.Name}: WillProcess");
        return true;
    }

    public override ILPostProcessResult Process(ICompiledAssembly compiledAssembly)
    {
        OutputDebugString($"{compiledAssembly.Name}: Start patching...");

        var msgs = new System.Collections.Generic.List<Unity.CompilationPipeline.Common.Diagnostics.DiagnosticMessage>();

        try
        {
            var assembly = compiledAssembly.GetAssemblyDefinition();
            var rpcProcessor = new PhysicsPostProcessor(assembly.MainModule);
            var anythingChanged = rpcProcessor.Process(assembly.MainModule);
            if (!anythingChanged)
            {
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
        }
        catch (System.Exception e)
        {
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
    Type m_Rapier = typeof(RapierLoop);
    BindingFlags m_DefaultBindingFlags = BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public;

    // Rigidbody replacements
    readonly Dictionary<(string MethodName, int ParameterCount), string> m_RigidbodyReplacements = new()
    {
        { ("AddForce", 1), "AddForce" },
        { ("AddForce", 2), "AddForceWithMode" },
        { ("AddTorque", 1), "AddTorque" },
        { ("AddTorque", 2), "AddTorqueWithMode" },
        { ("AddRelativeTorque", 1), "AddRelativeTorque" },
        { ("AddRelativeTorque", 2), "AddRelativeTorqueWithMode" },
        { ("AddRelativeForce", 1), "AddRelativeForce" },
        { ("AddRelativeForce", 2), "AddRelativeForceWithMode" },
        { ("MovePosition", 1), "MovePosition" },
        { ("MoveRotation", 1), "MoveRotation" },
        { ("Move", 2), "Move" },
    };

    // Physics replacements
    readonly Dictionary<(string MethodName, int ParameterCount), string> m_PhysicsReplacements = new()
    {
        { ("Raycast", 2), "Raycast" },
    };


    public PhysicsPostProcessor(ModuleDefinition assemblyMainModule)
    {
        m_AssemblyMainModule = assemblyMainModule;
    }

    // Replaces Calls to Physics Apis with Rapier Physics Apis
    public bool Process(ModuleDefinition assemblyMainModule)
    {
        bool anythingChanged = false;

        foreach (var type in assemblyMainModule.Types)
        {
            foreach (var method in type.Methods)
            {
                if (!method.HasBody)
                    continue;

                var instructions = method.Body.Instructions;
                for (int i = 0; i < instructions.Count; i++)
                {
                    var instruction = instructions[i];

                    if (instruction.OpCode == OpCodes.Callvirt)
                    {
                        if (instruction.Operand is not MethodReference methodReference)
                            continue;

                        if (methodReference.DeclaringType.FullName == "UnityEngine.Rigidbody")
                        {
                            var methodName = methodReference.Name;
                            var paramCount = methodReference.Parameters.Count;

                            if (m_RigidbodyReplacements.TryGetValue((methodName, paramCount), out var replacementMethod))
                            {
                                InjectFunction(replacementMethod, ref instruction);
                                anythingChanged = true;
                            }
                        }
                    }
                    else if (instruction.OpCode == OpCodes.Call)
                    {
                        if (instruction.Operand is not MethodReference methodReference)
                            continue;

                        if (methodReference.DeclaringType.FullName == "UnityEngine.Physics")
                        {
                            if (m_PhysicsReplacements.TryGetValue((methodReference.Name, methodReference.Parameters.Count), out var replacementMethod))
                            {
                                InjectFunction(replacementMethod, ref instruction);
                                anythingChanged = true;
                            }
                        }
                    }
                }
            }
        }

        return anythingChanged;
    }

    private void InjectFunction(string functionToInjectName, ref Instruction instruction)
    {
        var method = m_Rapier.GetMethod(functionToInjectName, m_DefaultBindingFlags);
        var newMethodReference = m_AssemblyMainModule.ImportReference(method);
        instruction.Operand = newMethodReference;
        Patch.OutputDebugString($"Method: {GetMethodSignature((MethodReference)instruction.Operand)} -> {GetMethodSignature(newMethodReference)}");
    }

    public static string GetMethodSignature(MethodReference methodReference)
        => $"{methodReference.DeclaringType.FullName}.{methodReference.Name}({string.Join(", ", methodReference.Parameters.Select(p => p.ParameterType.FullName))})";
}