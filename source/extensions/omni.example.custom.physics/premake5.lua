-- Setup the extension.
local ext = get_current_extension_info()
project_ext(ext)

-- Link folders that should be packaged with the extension.
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
}

-- Build the C++ plugin that will be loaded by the extension.
project_ext_plugin(ext, "omni.example.custom.physics.plugin")
    add_files("include", "include/omni/example/custom/physics")
    add_files("source", "plugins/omni.example.custom.physics")
    includedirs {
        root.."/include",
        "plugins/omni.example.custom.physics",
        "%{target_deps}/cuda",
        "%{target_deps}/nv_usd/release/include",
        "%{target_deps}/usd_ext_physics/%{cfg.buildcfg}/include",
        "%{target_deps}/pybind11/include",
        "%{target_deps}/python/include",
        "%{target_deps}/tinyxml2/include",
        "%{target_deps}/omni_physics/include",
        "%{target_deps}/client-library/include",
        "%{target_deps}/rtx_plugins/include",
        "%{target_deps}/carb_sdk_plugins/include",
        bin_dir.."/kit/extscore/usdrt.scenegraph/include",
        bin_dir.."/kit/dev/gsl/include",
        bin_dir.."/kit/dev/fabric/include",
        bin_dir.."/kit/python/include/python3.10",
        bin_dir.."/kit/dev/include",
        bin_dir.."/kit_sdk_%{cfg.buildcfg}/include",
        root.."/mujoco/include",
     }
    libdirs { "%{target_deps}/nv_usd/release/lib",
            root.."/mujoco/lib",
            root.."/mujoco/lib/Release",
            root.."/_build/target-deps/tinyxml2/lib",
            root.."/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/lib",
            bin_dir.."/kit/exts/omni.usd.core/bin", --omni.usd
            bin_dir.."/kit/exts/plugins",
        }

    links {"carb", "arch", "gf", "sdf", "tf", "vt", "pcp",
    "usd", "usdGeom", "usdUtils", "omni.usd", "usdPhysics", "physxSchema",
    "mujoco", "tinyxml2"}

    defines { "NOMINMAX", "NDEBUG" }
    runtime "Release"
    rtti "On"

    filter { "system:linux" }
        exceptionhandling "On"
        staticruntime "Off"
        cppdialect "C++17"
        includedirs { "%{target_deps}/python/include/python3.7m" }
        buildoptions { "-D_GLIBCXX_USE_CXX11_ABI=0 -Wno-error -Wno-deprecated-declarations -Wno-deprecated -Wno-unused-variable -pthread -lstdc++fs -Wno-undef -std=c++17" }
        linkoptions { "-Wl,--disable-new-dtags -Wl,-rpath,%{target_deps}/nv_usd/release/lib:%{target_deps}/python/lib:%{root}/mujoco/lib/" }
        filter { "system:windows" }
        buildoptions { "/wd4244 /wd4305 /wd4530 /wd4996" }
    filter {}

-- Build Python bindings that will be loaded by the extension.
project_ext_bindings {
    ext = ext,
    project_name = "omni.example.custom.physics.python",
    module = "_custom_physics_bindings",
    src = "bindings/python/omni.example.custom.physics",
    target_subdir = "omni/example/custom/physics"
}

    includedirs {
        root.."/include",
        "%{target_deps}/carb_sdk_plugins/include",
        bin_dir.."/kit/dev/include",
    }

    repo_build.prebuild_link {
        { "python/impl", ext.target_dir.."/omni/example/custom/physics/impl" },
        { "python/tests", ext.target_dir.."/omni/example/custom/physics/tests" },
    }


    repo_build.prebuild_copy {
        { "%{root}/mujoco/bin/Release/mujoco.dll**", ext.target_dir.."/bin/" },
        { "%{root}/_build/target-deps/usd_ext_physics/release/lib/physxSchema.dll**", ext.target_dir.."/bin/" },
        { "%{root}/mujoco/lib/libmujoco.so**", ext.target_dir.."/bin/" }
    }
