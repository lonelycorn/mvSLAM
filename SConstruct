import os
import mvSLAM_dependency # external dependencies

def collect_lib_dependencies(internal_libs, external_libs):
    result = []
    if (internal_libs):
        result.extend(internal_libs)
    if (external_libs):
        for mvSLAM_alias in external_libs:
            real_dep = mvSLAM_dependency.all_dependency[mvSLAM_alias]
            result.extend(real_dep)
    return result

def collect_obj_files(env, source, internal_modules):
    result = []
    if (source):
        result.extend([env.Object(s) for s in source])
    if (internal_modules):
        for module_name in internal_modules:
            result.extend(env[module_name])
    return result

def build_and_install_mvSLAM_library(env, target, source):
    """
    static libraries; no need to define all symbols.
    """
    source = collect_obj_files(env, source, None)
    build_lib = env.StaticLibrary(target, source)
    install_lib = env.Install("#/variant-dir/lib", build_lib)
    return install_lib

def build_and_install_mvSLAM_program(env, target, source,
                                     internal_modules=None,
                                     internal_libs=None, external_libs=None):
    source = collect_obj_files(env, source, internal_modules)
    libs = collect_lib_dependencies(internal_libs, external_libs)
    env1 = env.Clone()
    env1.Append(LIBS=libs)
    build_util = env1.Program(target, source)
    install_util = env1.Install('#/result/', build_util)
    return install_util

def build_and_run_mvSLAM_unit_test(env, target, source,
                                   internal_modules=None,
                                   internal_libs=None, external_libs=None):
    source = collect_obj_files(env, source, internal_modules)
    libs = collect_lib_dependencies(internal_libs, external_libs)
    env1 = env.Clone()
    env1.Append(LIBS=libs)
    build_unit_test = env1.Program(target, source)
    assert(len(build_unit_test) == 1)
    build_unit_test = build_unit_test[0]
    exe_dir = os.path.dirname(build_unit_test.path)
    exe_name = build_unit_test.name
    tag_name = "%s.passed" % (exe_name)
    run_unit_test = env1.Command(tag_name, \
                                 exe_name, \
                                 "cd %s;"\
                                 "if ./%s; then touch %s;" \
                                 "else rm %s &> /dev/null;"\
                                 "fi"\
                                 % (exe_dir, exe_name, tag_name, tag_name))
    return run_unit_test

# initialize the environment
env = Environment()
env.AddMethod(build_and_install_mvSLAM_library, 'mvSLAM_Library')
env.AddMethod(build_and_install_mvSLAM_program, 'mvSLAM_Program')
env.AddMethod(build_and_run_mvSLAM_unit_test, 'mvSLAM_UnitTest')
env.Append(CCFLAGS="-g -Wall -Werror -Wno-deprecated -pthread")
env.Append(CXXFLAGS="-std=c++11")
env.Append(CPPPATH=[
    "#/source/",
    "/usr/include/pcl-1.7/",
    "/usr/include/eigen3/",
    "/usr/include/vtk-6.2/",
    "/usr/include/boost/",
])
env.Append(CPPDEFINES=[
    # FIXME: Eigen's SVD may not be as stable as OpenCV's implementation.
    "USE_OPENCV_SVD",
    "USE_OPENCV_ESSENTIAL_MATRIX",
    # FIXME: The following two defines disable the vectorization, so that we don't need
    # to worry about Eigen's special requirement on memory alignment. However this would
    # slow down the system.
    "EIGEN_DONT_VECTORIZE",
    "EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT",
])
env.Append(LIBPATH=[
    "/usr/lib",
    "/usr/local/lib",
    "#/variant-dir/lib"
])


all_targets = []

# load all source targets
internal_libs, internal_modules = SConscript(
    '#/source/SConscript',
    variant_dir='#/variant-dir', # VariantDir
    src_dir='#/', # re-root
    exports=['env']
)
all_targets += internal_libs
# register internal modules. FIXME: these are source files
for (module_name, module_files) in internal_modules.items():
    env[module_name] = module_files


# load all tests
targets = SConscript(
    "#/test/SConscript",
    variant_dir='#/.test', # VariantDir
    src_dir="./", # re-root
    exports=['env']
)
all_targets += targets

# load all utilities
targets = SConscript(
    "#/utility/SConscript",
    variant_dir='#/variant-dir', # VariantDir
    src_dir="./", # re-root
    exports=['env']
)
all_targets += targets

Default(all_targets)
