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
        "include",
        "plugins/omni.example.custom.physics",
        "%{target_deps}/cuda",
        "%{target_deps}/nv_usd/release/include",
        "%{target_deps}/usd_ext_physics/%{cfg.buildcfg}/include",
        "%{target_deps}/pybind11/include",
        "%{target_deps}/python/include",
        "%{target_deps}/tinyxml2/include",
        "%{target_deps}/omni_physics/include",
        bin_dir.."/kit/extscore/usdrt.scenegraph/include",
        bin_dir.."/kit/dev/gsl/include",
        bin_dir.."/kit/dev/fabric/include",
        -- "/home/mrakhsha/Documents/Repos/mujoco/include",
        root.."/mujoco/include",
     }
    libdirs { "%{target_deps}/nv_usd/release/lib",
            root.."/mujoco-build/lib/release",
            root.."/_build/target-deps/tinyxml2/lib",
            root.."/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/lib",
        }

    links { "arch", "gf", "sdf", "tf", "usd", "usdGeom", "usdUtils", "carb", "mujoco", "tinyxml2", "usdPhysics"}
    defines { "NOMINMAX", "NDEBUG" }
    runtime "Release"
    rtti "On"

    filter { "system:linux" }
        exceptionhandling "On"
        staticruntime "Off"
        cppdialect "C++17"
        includedirs { "%{target_deps}/python/include/python3.7m" }
        buildoptions { "-D_GLIBCXX_USE_CXX11_ABI=0 -Wno-error -Wno-deprecated-declarations -Wno-deprecated -Wno-unused-variable -pthread -lstdc++fs -Wno-undef -std=c++17" }
        linkoptions { "-Wl,--disable-new-dtags -Wl,-rpath,%{target_deps}/nv_usd/release/lib:%{target_deps}/python/lib:" }
    filter { "system:windows" }
        buildoptions { "/wd4244 /wd4305 /wd4530" }
    filter {}

-- Build Python bindings that will be loaded by the extension.
project_ext_bindings {
    ext = ext,
    project_name = "omni.example.custom.physics.python",
    module = "_custom_physics_bindings",
    src = "bindings/python/omni.example.custom.physics",
    target_subdir = "omni/example/custom/physics"
}
    includedirs { "include" }
    repo_build.prebuild_link {
        { "python/impl", ext.target_dir.."/omni/example/custom/physics/impl" },
        { "python/tests", ext.target_dir.."/omni/example/custom/physics/tests" },
    }
