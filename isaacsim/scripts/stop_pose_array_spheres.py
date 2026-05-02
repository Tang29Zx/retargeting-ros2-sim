visualizer = globals().get("_hybrik_pose_visualizer")

if visualizer is None:
    print("HybrIK pose visualizer is not running.")
else:
    visualizer.stop()
    _hybrik_pose_visualizer = None
    print("Stopped HybrIK pose visualizer.")
