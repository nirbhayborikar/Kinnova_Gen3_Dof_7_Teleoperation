


moveit_config = (
    MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
    .robot_description(mappings=description_arguments)
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .planning_scene_monitor(
        publish_robot_description=True, publish_robot_description_semantic=True
    )
    .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
    .to_moveit_configs()
)