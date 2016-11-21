
def build_and_install_mvSLAM_library(env, libname, sources):
    libs = env["mvSLAM_external_dependency"]
    #build_lib = env.SharedLibrary(libname, sources, LIBS=libs)
    build_lib = env.StaticLibrary(libname, sources, LIBS=libs)
    install_lib = env.Install('#/variant-dir/lib/', build_lib)
    return install_lib

def build_and_install_mvSLAM_program(env, target, source):
    env1 = env.Clone()
    internal_libs = [
                "mvSLAM_base",
                "mvSLAM_os",
                "mvSLAM_vision",
                ]
    libs = internal_libs + env["mvSLAM_external_dependency"]
    env1.Append(LIBS=libs)
    env1.Append(LIBPATH=["#/variant-dir/lib",
                         "/usr/local/lib"])
    build_util = env1.Program(target, source)
    install_util = env1.Install('#/result/', build_util)
    return install_util

def build_and_run_mvSLAM_unit_test(env, target, source):
    build_unit_test = env.Program(target, source)
    exe_dir = build_unit_test.dir
    exe_name = build_unit_test.name
    tag_name = "%s.passed" % (executable_name)
    run_unit_test = env.Command("cd %s;"\
                                "rm -rf %s &> /dev/null;"\
                                "if ./%s; then touch %s; fi" \
                                % (exe_dir, tag_name, exe_name, tag_name))
    return run_unit_test 
    
MVSLAM_EXTERNAL_DEPENDENCY = [
                #===============#
                #     boost     #
                #===============#
                "boost_system",
 
                #=============#
                #     pcl     #
                #=============#
                #"pcl_apps",
                "pcl_common",
                #"pcl_features",
                #"pcl_filters",
                #"pcl_io_ply",
                "pcl_io",
                #"pcl_kdtree",
                #"pcl_keypoints",
                #"pcl_octree",
                #"pcl_outofcore",
                #"pcl_people",
                #"pcl_recognition",
                #"pcl_registration",
                #"pcl_sample_consensus",
                #"pcl_search",
                #"pcl_segmentation",
                #"pcl_surface",
                #"pcl_tracking",
                "pcl_visualization",
 
                #=============#
                #     vtk     #
                #=============#
                #"vtkalglib",
                #"vtkCharts",
                "vtkCommon",
                #"vtkDICOMParser",
                #"vtkexoIIc",
                "vtkFiltering",
                #"vtkftgl",
                #"vtkGenericFiltering"
                #"vtkGeovis",
                "vtkGraphics",
                #"vtkHybrid",
                #"vtkImaging",
                #"vtkInfovis",
                #"vtkIO",
                #"vtkmetaio",
                #"vtkParallel",
                #"vtkproj4",
                #"vtkQtChart",
                "vtkRendering",
                #"vtksys",
                #"vtkViews",
                #"vtkVolumeRendering",
                #"vtkWidgets",
 
                #================#
                #     opencv     #
                #================#
                #"opencv_stitching",
                #"opencv_superres",
                #"opencv_videostab",
                #"opencv_adas",
                #"opencv_bgsegm",
                #"opencv_bioinspired",
                #"opencv_ccalib",
                #"opencv_datasets",
                #"opencv_face",
                #"opencv_latentsvm",
                #"opencv_objdetect",
                #"opencv_line_descriptor",
                #"opencv_optflow",
                #"opencv_reg",
                #"opencv_rgbd",
                #"opencv_saliency",
                #"opencv_surface_matching",
                #"opencv_text",
                #"opencv_tracking",
                #"opencv_xfeatures2d",
                #"opencv_calib3d",
                #"opencv_features2d",
                #"opencv_shape",
                #"opencv_video",
                #"opencv_ml",
                #"opencv_flann",
                #"opencv_ximgproc",
                #"opencv_xobjdetect",
                #"opencv_xphoto",
                "opencv_highgui",
                #"opencv_videoio",
                "opencv_imgcodecs",
                #"opencv_photo",
                #"opencv_imgproc",
                "opencv_core",
                #"opencv_hal",
                #================#
                #     system     #
                #================#
                "m",
                "rt",
                "pthread",
                ]

# initialize the environment
env = Environment()
env["CCFLAGS"] = "-g -Wall -Werror -Wno-deprecated -pthread"
env["CXXFLAGS"] = "-std=c++11"
env["mvSLAM_external_dependency"] = MVSLAM_EXTERNAL_DEPENDENCY
env["CPPPATH"] = ["#/source/", 
                  "/usr/include/pcl-1.7/",
                  "/usr/include/eigen3/",
                  "/usr/include/vtk-5.8/",
                  "/usr/include/boost/",
                  "/usr/include/flann/",
                 ]

env.AddMethod(build_and_run_mvSLAM_unit_test, 'mvSLAM_UnitTest')
env.AddMethod(build_and_install_mvSLAM_library, 'mvSLAM_Library')
env.AddMethod(build_and_install_mvSLAM_program, 'mvSLAM_Program')

all_targets = []

# load all source targets 
targets = SConscript('source/SConscript',
                     variant_dir='./variant-dir', # VariantDir
                     src_dir='./', # re-root
                     exports=['env']
                    )
all_targets += targets

# load all tests
targets = SConscript("test/SConscript",
                     variant_dir='./variant-dir', # VariantDir
                     src_dir="./", # re-root
                     exports=['env']
                    )
all_targets += targets

Default(all_targets)
