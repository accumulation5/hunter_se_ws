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


-- 建议创建一个 custom_assets_writer_3d.lua
-- options = {
--   tracking_frame = "base_link",
--   pipeline = {
--     {
--       action = "min_max_range_filter",
--       min_range = 1.,
--       max_range = 50.,
--     },
--     --  体素下采样：用 10cm 体素合并点云，大幅减少点数
--     {
--       action = "write_hybrid_grid",
--       voxel_size = 0.10,  -- 10cm 体素，结构化场景足够
--       range_data_inserter = {
--         insert_free_space = false,
--         hit_probability = 0.55,
--         miss_probability = 0.49,
--         num_free_space_voxels = 0,
--       },
--       filename = "downsampled_grid",
--     },
--     --  固定比例采样：只保留 10%~20% 的点
--     -- {
--     --   action = "fixed_ratio_sampler",
--     --   sampling_ratio = 0.1,  -- 只保留 10%
--     -- },
--     {
--       action = "write_ply",
--       filename = "points.ply",
--     },
--     {
--       action = "write_pcd",
--       filename = "points.pcd",
--     },
--   }
-- }