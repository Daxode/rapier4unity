use anyhow::Result;
use phf::{Map, phf_map};
use proc_macro2::TokenTree::{Ident, Punct};
use quote::ToTokens;
use std::env::args;
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use syn::visit::Visit;

// static type translation hashmap
static RUST_TYPE_TO_CSHARP: Map<&'static str, &'static str> = phf_map! {
    "SerializableColliderHandle" => "ColliderHandle",
    "SerializableRigidBodyType" => "RigidBodyType",
    "SerializableRigidBodyHandle" => "RigidBodyHandle",
    "SerializableImpulseJointHandle" => "ImpulseJointHandle",
    "SerializableCollisionEvent" => "CollisionEvent",
    "RaycastHit" => "RapierRaycastHit",
    "Vector3<float>" => "float3",
    "Vector2<float>" => "float2",
    "u32" => "uint",
    "f32" => "float",
    "i32" => "int",
    "usize" => "UIntPtr",
    // Add more entries
};

fn main() -> Result<()> {
    // find input folder
    let folder = args().nth(1).expect("Please provide an input folder.");
    let files = get_rust_files(&folder)?;

    // open file to write or create if it doesn't exist
    let file = OpenOptions::new()
        .append(false)
        .write(true)
        .open("../../Runtime/RapierBinds.cs")
        .or_else(|err| {
            if err.kind() == std::io::ErrorKind::NotFound {
                File::create("../../Runtime/RapierBinds.cs")
            } else {
                Err(err)
            }
        });

    let mut writer = BufWriter::new(file?);
    // write to file
    writeln!(&mut writer, "//#define DISABLE_DYNAMIC_RAPIER_LOAD")?;
    writeln!(&mut writer, "")?;
    writeln!(&mut writer, "using System;")?;
    writeln!(&mut writer, "using System.IO;")?;
    writeln!(&mut writer, "using System.Runtime.InteropServices;")?;
    writeln!(&mut writer, "using Packages.rapier4unity.Runtime;")?;
    writeln!(&mut writer, "using Unity.Collections.LowLevel.Unsafe;")?;
    writeln!(&mut writer, "using Unity.Burst;")?;
    writeln!(&mut writer, "using UnityEngine;")?;
    writeln!(&mut writer, "using Unity.Mathematics;")?;
    writeln!(&mut writer, "")?;

    writeln!(
        &mut writer,
        "#if UNITY_EDITOR && !DISABLE_DYNAMIC_RAPIER_LOAD"
    )?;
    writeln!(&mut writer, "[UnityEditor.InitializeOnLoad]")?;
    writeln!(&mut writer, "#endif")?;

    writeln!(&mut writer, "")?;
    writeln!(&mut writer, "[BurstCompile]")?;
    writeln!(&mut writer, "internal static unsafe class RapierBindings")?;
    writeln!(&mut writer, "{{")?;
    writeln!(&mut writer, "#if UNITY_STANDALONE_OSX")?;
    writeln!(
        &mut writer,
        "\tprivate const string DllName = \"librapier_c_bind.dylib\";"
    )?;
    writeln!(
        &mut writer,
        "\tconst string k_DLLPath = \"Packages/rapier4unity/build_bin/librapier_c_bind.dylib\";"
    )?;
    writeln!(&mut writer, "#else")?;
    writeln!(
        &mut writer,
        "\tprivate const string DllName = \"rapier_c_bind\";"
    )?;
    writeln!(
        &mut writer,
        "\tconst string k_DLLPath = \"Packages/rapier4unity/build_bin/rapier_c_bind.dll\";"
    )?;
    writeln!(&mut writer, "#endif")?;
    writeln!(
        &mut writer,
        "\tprivate const CallingConvention Convention = CallingConvention.Cdecl;"
    )?;

    let mut raw_function_ptrs = Vec::new();
    let mut load_calls = Vec::new();

    for file in files {
        println!("reading file: {}", file);

        // Read the Rust source file
        let code = std::fs::read_to_string(file)?;

        // Parse the code into an AST
        let ast = syn::parse_file(&code)?;

        // Visitor to find the target function and its arguments
        let mut visitor = FunctionVisitor {
            function_names: Vec::new(),
            function_ptr_signatures: Vec::new(),
            extern_function_signatures: Vec::new(),
        };
        visitor.visit_file(&ast);
        if visitor.function_names.is_empty() {
            continue;
        }

        for i in 0..visitor.function_names.len() {
            load_calls.push(format!(
                "{} = NativeLoader.GetFunction(loaded_lib, \"{}\");",
                to_camel_case(&visitor.function_names[i]),
                visitor.function_names[i]
            ));
            raw_function_ptrs.push(format!(
                "public IntPtr {};",
                to_camel_case(&visitor.function_names[i])
            ));
        }

        writeln!(
            &mut writer,
            "#if UNITY_EDITOR && !DISABLE_DYNAMIC_RAPIER_LOAD"
        )?;
        for i in 0..visitor.function_names.len() {
            writeln!(&mut writer, "\t{}", visitor.function_ptr_signatures[i])?;
        }
        writeln!(&mut writer, "#else")?;
        for i in 0..visitor.function_names.len() {
            writeln!(&mut writer, "\t{}", visitor.extern_function_signatures[i])?;
        }
        writeln!(&mut writer, "#endif")?;
    }
    // stuff
    writeln!(
        &mut writer,
        r#"
#if UNITY_EDITOR && !DISABLE_DYNAMIC_RAPIER_LOAD
    // C# -> Rust
    [StructLayout(LayoutKind.Sequential, Size=sizeof(ulong) * 512)]
    struct UnmanagedData
    {{
        IntPtr loaded_lib;
        public void load_calls()
        {{
            if (loaded_lib==IntPtr.Zero)
            {{
                loaded_lib = NativeLoader.LoadLibrary(Path.GetFullPath(k_DLLPath));
                if (loaded_lib == IntPtr.Zero)
                {{
                    var errorMsg = NativeLoader.GetLastErrorString();
                    Console.WriteLine($"Failed to load library: {{errorMsg}}");
                    return;
                }}
            }}

            // Load function pointers
            {}
        }}

        // Raw function pointers
        {}

        // Rust -> C# data
        public FunctionsToCallFromRust functionsToCallFromRust;

        public void unload_calls()
        {{
            if (loaded_lib != IntPtr.Zero)
                NativeLoader.FreeLibrary(loaded_lib);
            loaded_lib = IntPtr.Zero;
            Debug.Log($"RapierBindingCalls Unloaded");
        }}
        public bool IsLoaded => loaded_lib != IntPtr.Zero;
    }}
    public static bool IsAvailable => data.Data.IsLoaded;
    public static void LoadCalls()
    {{
        data.Data.load_calls();
        init_rapier4unity();
        Debug.Log($"RapierBindingCalls Loaded");
    }}

    [BurstCompile]
    static void init_rapier4unity(){{
        data.Data.functionsToCallFromRust.Init();
        Init((FunctionsToCallFromRust*)UnsafeUtility.AddressOf(ref data.Data.functionsToCallFromRust));
    }}

    public static void UnloadCalls() => data.Data.unload_calls();

    struct SpecialKey {{}}
    static readonly SharedStatic<UnmanagedData> data = SharedStatic<UnmanagedData>.GetOrCreate<SpecialKey>();
#else
    // No dynamic loading, empty functions
    public static bool IsAvailable => true;
    public static void LoadCalls(){{}}
    public static void UnloadCalls(){{}}

    // Called during InitializeOnLoad
	static RapierBindings()
	{{
		var functionsToCallFromRust = new FunctionsToCallFromRust();
		functionsToCallFromRust.Init();
		Init(&functionsToCallFromRust);
	}}
#endif

    // Rust -> C#
    [BurstCompile]
    [StructLayout(LayoutKind.Sequential)]
    public struct FunctionsToCallFromRust {{
        public IntPtr unityLogPtr;

        public void Init()
        {{
            unityLogPtr = UnityRapierBridge.GetDefaultUnityLogger();
        }}
    }}
"#,
        load_calls.join("\n\t\t\t"),
        raw_function_ptrs.join("\n\t\t")
    )?;
    writeln!(&mut writer, "}}")?;
    writer.flush()?;

    Ok(())
}

// turn snake_case to PascalCase
fn to_pascal_case(name: &str) -> String {
    let mut result = String::new();
    let mut capitalize = true;
    for c in name.chars() {
        if c == '_' {
            capitalize = true;
        } else if capitalize {
            result.push(c.to_ascii_uppercase());
            capitalize = false;
        } else {
            result.push(c);
        }
    }
    result
}

// turn snake_case to camelCase
fn to_camel_case(name: &str) -> String {
    let mut result = String::new();
    let mut capitalize = false;
    for c in name.chars() {
        if c == '_' {
            capitalize = true;
        } else if capitalize {
            result.push(c.to_ascii_uppercase());
            capitalize = false;
        } else {
            result.push(c);
        }
    }
    result
}

struct FunctionVisitor {
    function_names: Vec<String>,
    function_ptr_signatures: Vec<String>,
    extern_function_signatures: Vec<String>,
}

fn is_extern_c(sig: &syn::Signature) -> bool {
    sig.abi
        .as_ref()
        .and_then(|abi| abi.name.as_ref())
        .map(|lit_str| lit_str.value() == "C")
        .unwrap_or(false)
}

impl<'a> Visit<'_> for FunctionVisitor {
    fn visit_item_fn(&mut self, node: &syn::ItemFn) {
        // Check if this is the target function
        //self.function_name = node.sig.ident.to_string();

        if !is_extern_c(&node.sig) {
            syn::visit::visit_item_fn(self, node);
            return;
        }

        // Extract each argument's name and type
        let pascal_cased_name = to_pascal_case(&node.sig.ident.to_string());
        let camel_cased_name = to_camel_case(&node.sig.ident.to_string());
        let mut types = Vec::new();
        let mut parameter_list = Vec::new();
        let mut names = Vec::new();
        let mut return_type = String::new();
        if let syn::ReturnType::Type(_, return_typee) = &node.sig.output {
            return_type.push_str(get_typename(return_typee).as_str());
        } else {
            return_type.push_str("void");
        }

        for arg in &node.sig.inputs {
            if let syn::FnArg::Typed(pat_type) = arg {
                // Get argument name
                let name = match &*pat_type.pat {
                    syn::Pat::Ident(ident) => ident.ident.to_string(),
                    _ => "<unnamed>".to_string(),
                };

                // Get argument type as a string
                let some_type = get_typename(&pat_type.ty);

                //println!("{:?}", some_type);
                types.push(format!("{}", some_type));
                let param_name = to_camel_case(name.as_str());
                parameter_list.push(format!("{} {}", some_type, param_name));
                names.push(param_name);
            }
        }
        types.push(return_type.to_string());

        self.function_names.push(node.sig.ident.to_string());
        self.function_ptr_signatures.push(format!(
            "public static {} {}({}) => ((delegate* unmanaged[Cdecl]<{}>) data.Data.{})({});",
            return_type,
            pascal_cased_name,
            parameter_list.join(", "),
            types.join(", "),
            camel_cased_name,
            names.join(", ")
        ));
        self.extern_function_signatures.push(format!("[DllImport(DllName, CallingConvention = Convention, EntryPoint=\"{}\")]\n\tpublic static extern unsafe {} {}({});", node.sig.ident, return_type, pascal_cased_name, parameter_list.join(", ")));
        // Output the result
        // println!(
        //     "found '{}', with argument(s): {}",
        //     node.sig.ident,
        //     self.args.join(", ")
        // );

        // Continue traversing the AST
        syn::visit::visit_item_fn(self, node);
    }
}

fn get_typename(ty: &syn::Type) -> String {
    let mut some_type = String::new();
    let mut is_parsing_pointer = false;
    //let mut is_parsing_typearg = false;
    let mut pending_pointers = 0;
    for token in ty.to_token_stream() {
        if let Punct(punct) = token {
            if punct.as_char() == '*' {
                is_parsing_pointer = true;
            } else if punct.as_char() == '<' || punct.as_char() == '>' {
                some_type.push(punct.as_char());
            }
        } else if let Ident(punct) = token {
            if is_parsing_pointer {
                assert!(punct == "mut" || punct == "const");
                pending_pointers += 1;
                is_parsing_pointer = false;
            } else {
                if let Some(csharp_type) = RUST_TYPE_TO_CSHARP.get(&punct.to_string().as_str()) {
                    some_type.push_str(csharp_type);
                } else {
                    some_type.push_str(&format!("{}", punct));
                }
            }
        }

        //some_type.push_str(&format!("{}", token));
    }

    if let Some(csharp_type) = RUST_TYPE_TO_CSHARP.get(&some_type) {
        some_type = csharp_type.to_string();
    }

    if pending_pointers > 0 {
        for _ in 0..pending_pointers {
            some_type.push_str("*");
        }
    }

    some_type
}

// get all rust files in the input folder
fn get_rust_files(folder: &str) -> Result<Vec<String>> {
    let mut files = Vec::new();
    for entry in std::fs::read_dir(folder)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_file() && path.extension() == Some("rs".as_ref()) {
            // get absolute path from relative path
            let path = path.canonicalize()?;
            files.push(path.to_string_lossy().to_string());
        }
    }
    Ok(files)
}
