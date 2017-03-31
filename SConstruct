import os
import mvSLAM_dependency

def build_and_install_mvSLAM_program(env, target, source):
    env1 = env.Clone()
    libs = env1["mvSLAM_external_dependency"]
    env1.Append(LIBS=libs)
    env1.Append(LIBPATH=["/usr/lib",
                         "/usr/local/lib"])
    module_source = env1["MVSLAM_MODULE_SOURCE"]
    source.extend(module_source)
    build_util = env1.Program(target, source)
    install_util = env1.Install('#/result/', build_util)
    return install_util

def build_and_run_mvSLAM_unit_test(env, target, source):
    env1 = env.Clone()
    libs = env1["mvSLAM_external_dependency"]
    env1.Append(LIBS=libs)
    env1.Append(LIBPATH=["/usr/lib",
                         "/usr/local/lib"])
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
                                 "else [ -f %s ] && rm %s;"\
                                 "fi"\
                                 % (exe_dir, exe_name, tag_name, tag_name, tag_name))
    return run_unit_test 

# initialize the environment
env = Environment()
env["CCFLAGS"] = "-g -Wall -Werror -Wno-deprecated -pthread"
env["CXXFLAGS"] = "-std=c++11"
env["mvSLAM_external_dependency"] = mvSLAM_dependency.all_dependency
env["CPPPATH"] = ["#/source/", 
                  "/usr/include/pcl-1.7/",
                  "/usr/include/eigen3/",
                  "/usr/include/vtk-6.2/",
                  "/usr/include/boost/",
                 ]
env.Append(CPPDEFINES=["USE_OPENCV_SVD",
                       "USE_OPENCV_ESSENTIAL_MATRIX",
                      ])

env.AddMethod(build_and_install_mvSLAM_program, 'mvSLAM_Program')
env.AddMethod(build_and_run_mvSLAM_unit_test, 'mvSLAM_UnitTest')

all_targets = []

# load all source targets 
targets = SConscript('#/source/SConscript',
                     variant_dir='#/variant-dir', # VariantDir
                     src_dir='#/', # re-root
                     exports=['env']
                    )
env["MVSLAM_MODULE_SOURCE"] = targets
all_targets += targets

# load all tests
targets = SConscript("#/test/SConscript",
                     variant_dir='./.unit-test', # VariantDir
                     src_dir="./", # re-root
                     exports=['env']
                    )
all_targets += targets

# load all utilities
targets = SConscript("#/utility/SConscript",
                     variant_dir='./variant-dir', # VariantDir
                     src_dir="./", # re-root
                     exports=['env']
                    )
all_targets += targets

Default(all_targets)
