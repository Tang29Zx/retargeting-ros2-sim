visualizer = globals().get("_retarget_keypoint_visualizer")

if visualizer is None:
    print("Retarget keypoint visualizer is not running.")
else:
    visualizer.stop()
    _retarget_keypoint_visualizer = None
    print("Stopped retarget keypoint visualizer.")
