"""Generate isolated Nav2 test profile/map from existing algorithm configs.
Runtime overrides live ONLY under pid_tuning. No algorithm files edited.
"""
from pathlib import Path
import yaml,numpy as np
ROOT=Path(__file__).resolve().parent;SIM=ROOT.parent
profile=yaml.safe_load((SIM/'src/sentry_velocity/velocity_control/config/algorithm_profiles/navigationros2.yaml').read_text())
keep=['fake_vel_transform','ec_odometry_time_adapter','navigation_to_chassis','map_server','map_lifecycle_manager','planner_server','controller_server','bt_navigator','behavior_server','navigation_lifecycle_manager']
actions=[]
for a in profile['actions']:
 if a.get('name') not in keep and not (a['type']=='include' and 'fake_vel_transform' in a.get('launch','')):continue
 if a.get('name')=='map_server':a['parameters']=[{'yaml_filename':str(ROOT/'test_map.yaml')}]
 if a.get('name') in ['controller_server','planner_server']:a['parameters'].append(str(ROOT/'test_costmaps.yaml'))
 if a.get('name')=='controller_server':a['parameters'].append({'PurePursuit.desired_linear_vel':2.0,'goal_checker.xy_goal_tolerance':.05})
 # Fixture runs synchronously with physics, no real-world 80ms communication delay.
 if a.get('name')=='navigation_to_chassis':a['parameters']=[{'navigation_delay_seconds':0.0}]
 actions.append(a)
profile['actions']=actions;profile['required_packages']=[p for p in profile['required_packages'] if p not in ['small_point_lio','sentry_decision','pointcloud_segmentation','rviz2']]
(ROOT/'nav2_test_profile.yaml').write_text(yaml.safe_dump(profile,sort_keys=False))
# 60x60m arena, 5cm cells; obstacle raster corresponds exactly to inserted boxes.
w=h=1200;res=.05;origin=(-10,-10);image=np.full((h,w),254,np.uint8)
import json
obstacles=json.loads((ROOT/'results/obstacles.json').read_text()) if (ROOT/'results/obstacles.json').exists() else []
for b in obstacles:
 x0=int((b['x']-b['width']/2-origin[0])/res);x1=int((b['x']+b['width']/2-origin[0])/res)
 y0=int((b['y']-b['width']/2-origin[1])/res);y1=int((b['y']+b['width']/2-origin[1])/res)
 image[h-y1:h-y0,x0:x1]=0
(ROOT/'test_map.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode()+image.tobytes())
(ROOT/'test_map.yaml').write_text(yaml.safe_dump(dict(image='test_map.pgm',resolution=res,origin=[*origin,0.0],negate=0,occupied_thresh=.65,free_thresh=.196)))
settings={}
for name,frame in [('global_costmap','map'),('local_costmap','odom')]:
 settings[name]={name:{'ros__parameters':{'use_sim_time':True,'global_frame':frame,'robot_base_frame':'base_link_fake','plugins':['static_layer','inflation_layer'],'static_layer':{'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True},'inflation_layer':{'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':.5,'cost_scaling_factor':3.0}}}}
(ROOT/'test_costmaps.yaml').write_text(yaml.safe_dump(settings));print(ROOT/'nav2_test_profile.yaml')
