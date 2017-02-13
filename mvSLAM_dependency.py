boost_dependency = [
            "boost_system",
        ]

pcl_dependency = [
            #"pcl_apps",
            "pcl_common",
            #"pcl_features",
            #"pcl_filters",
            #"pcl_io_ply",
            #"pcl_io",
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
        ]

vtk_dependency = [
#5.10
            #"vtkalglib",
            #"vtkCharts",
            #"vtkCommon",
            #"vtkDICOMParser",
            #"vtkexoIIc",
            #"vtkFiltering",
            #"vtkftgl",
            #"vtkGenericFiltering"
            #"vtkGeovis",
            #"vtkGraphics",
            #"vtkHybrid",
            #"vtkImaging",
            #"vtkInfovis",
            #"vtkIO",
            #"vtkmetaio",
            #"vtkParallel",
            #"vtkproj4",
            #"vtkQtChart",
            #"vtkRendering",
            #"vtksys",
            #"vtkViews",
            #"vtkVolumeRendering",
            #"vtkWidgets",
#6.2
            "vtkRenderingLOD-6.2",
            "vtkRenderingCore-6.2",
            "vtkFiltersSources-6.2",
            "vtkFiltersCore-6.2",
            "vtkFiltersGeometry-6.2",
            "vtkCommonExecutionModel-6.2",
            "vtkCommonCore-6.2",
            "vtkCommonDataModel-6.2",
            "vtkCommonMath-6.2",

        ]

gtsam_dependency = [
            #"gtsam_unstable",
            "gtsam",
            "tbb",
            "tbbmalloc",
        ]

opencv_dependency = [
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
            "opencv_calib3d",
            "opencv_features2d",
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
        ]

system_dependency = [
            "pthread",
            "rt",
            "m",
        ]

all_dependency = \
        boost_dependency + \
        pcl_dependency + \
        vtk_dependency + \
        gtsam_dependency + \
        opencv_dependency + \
        system_dependency

