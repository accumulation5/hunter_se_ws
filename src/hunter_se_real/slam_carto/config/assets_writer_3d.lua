-- sim_carto 3D 地图导出流水线：体素滤波 + PLY 输出

options = {
  tracking_frame = "map",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 100.,
    },
    {
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = 0.1,   -- 0.05~0.1 m 可调，越大越稀疏
    },
    {
      action = "write_ply",
      filename = "points.ply",
    },
  }
}

return options