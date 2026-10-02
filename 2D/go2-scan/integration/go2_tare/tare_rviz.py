"""RViz display setup for the selected real-robot exploration algorithm."""
import copy

def configure(config, algorithm, simulation=False):
    displays = config['Visualization Manager']['Displays']
    def toggle_ar(items):
        for item in items:
            if item.get('Name') == 'ARiADNE' or item.get('Topic') == '/frontier' or item.get('Marker Topic') == '/ariadne/bridge/adjusted_goal':
                item['Enabled'] = algorithm != 'tare'; item['Value'] = algorithm != 'tare'
            if 'Displays' in item:
                toggle_ar(item['Displays'])
    toggle_ar(displays)
    if algorithm == 'tare':
        def hide_short_map(items):
            for d in items:
                if d.get('Topic') == '/scan_map': d['Enabled']=False; d['Value']=False
                hide_short_map(d.get('Displays', []))
        hide_short_map(displays)
    def path(name, topic, color, enabled=True):
        return {'Class':'rviz/Path','Name':name,'Topic':topic,'Enabled':enabled,'Value':enabled,'Color':color,'Alpha':1,'Buffer Length':1,'Line Style':'Lines','Line Width':.04,'Pose Style':'None','Queue Size':2}
    def cloud(name, topic, enabled=False):
        return {'Class':'rviz/PointCloud2','Name':name,'Topic':topic,'Enabled':enabled,'Value':enabled,'Position Transformer':'XYZ','Color Transformer':'Intensity','Channel Name':'intensity','Autocompute Intensity Bounds':True,'Use rainbow':True,'Style':'Flat Squares','Size (m)':.10,'Size (Pixels)':3,'Alpha':1,'Decay Time':0,'Queue Size':1,'Selectable':False,'Use Fixed Frame':True}
    def marker(name,topic):
        return {'Class':'rviz/Marker','Name':name,'Marker Topic':topic,'Enabled':True,'Value':True,'Queue Size':2}
    terrain=cloud('Local terrain (height cost)','/tare/terrain_map',True)
    terrain.update({'Autocompute Intensity Bounds':False,'Min Intensity':0,'Max Intensity':.5})
    history=cloud('累计观测地图（非覆盖判定）','/tare/observed_map',True)
    history.update({'Color Transformer':'FlatColor','Color':'150; 150; 150','Alpha':.5,'Size (m)':.08})
    frontier=cloud('前沿（红色，保留最后快照）','/tare/display/frontiers',True)
    frontier.update({'Color Transformer':'FlatColor','Color':'255; 40; 40','Size (m)':.13})
    pending=cloud('未覆盖前沿（橙色）','/tare/display/uncovered_frontiers',True)
    pending.update({'Color Transformer':'FlatColor','Color':'255; 130; 0','Size (m)':.18})
    selected=cloud('计划访问观察点','/tare/display/selected_viewpoints',True)
    selected.update({'Color Transformer':'FlatColor','Color':'0; 255; 255','Style':'Spheres','Size (m)':.16})
    candidates=cloud('候选观察点（可选）','/tare/display/viewpoints')
    candidates.update({'Color Transformer':'FlatColor','Color':'80; 160; 255','Size (m)':.08})
    state={'Class':'rviz/MarkerArray','Name':'探索状态与目标标签','Marker Topic':'/tare/status_markers','Enabled':False,'Value':False,'Queue Size':1}
    surfaces=cloud('规划表面（非完整地图，可选）','/sensor_coverage_planner/planner_cloud')
    surfaces.update({'Color Transformer':'RGB8','Use rainbow':False})
    obstacles=cloud('障碍点云（红色）','/sensor_coverage_planner/collision_cloud',True)
    obstacles.update({'Color Transformer':'FlatColor','Color':'255; 40; 40','Use rainbow':False,'Size (m)':.08})
    blocked=cloud('碰撞视点（橙色）','/sensor_coverage_planner/viewpoint_in_collision_cloud_',True)
    blocked.update({'Color Transformer':'FlatColor','Color':'255; 150; 0','Use rainbow':False,'Size (m)':.15})
    entries=[history,frontier,pending,selected,candidates,state,terrain,cloud('Extended terrain (optional)','/tare/terrain_map_ext'),
      path('Global path','/sensor_coverage_planner/global_path','255; 170; 0'),
      path('Local exploration path','/sensor_coverage_planner/local_path','0; 255; 255'),
      {'Class':'rviz/PointStamped','Name':'TARE目标点（紫色）','Topic':'/tare/way_point','Enabled':True,'Value':True,'Color':'204; 41; 204','Alpha':1,'Radius':.15,'History Length':1},
      path('Goal sent to SCAN','/initial_path','100; 255; 100'),
      marker('Exploring subspaces','/sensor_coverage_planner/tare_visualizer/exploring_subspaces'),
      marker('Local planning horizon','/sensor_coverage_planner/tare_visualizer/local_planning_horizon'),
      surfaces,obstacles,blocked]
    by_topic = {d.get('Topic', d.get('Marker Topic')): d for d in entries}
    def take(topic, name, enabled=True):
        d = by_topic[topic]
        d.update(Name=name, Enabled=enabled, Value=enabled)
        return d
    targets = [take('/tare/way_point', '探索目标'),
               take('/initial_path', '下发目标'),
               take('/sensor_coverage_planner/global_path', '全局路径'),
               take('/sensor_coverage_planner/local_path', '局部路径')]
    progress = [take('/tare/display/frontiers', '有效前沿'),
                take('/tare/display/uncovered_frontiers', '待覆盖前沿'),
                take('/sensor_coverage_planner/tare_visualizer/exploring_subspaces', '待探索区域'),
                take('/tare/display/selected_viewpoints', '选中观察点', False)]
    maps = [take('/tare/observed_map', '观测地图'),
            take('/tare/terrain_map', '局部地形', False),
            take('/sensor_coverage_planner/collision_cloud', '障碍点云（红色）'),
            take('/sensor_coverage_planner/viewpoint_in_collision_cloud_', '碰撞视点（橙色）')]
    debug = [take('/tare/display/viewpoints', '候选观察点', False),
             take('/sensor_coverage_planner/tare_visualizer/local_planning_horizon', '规划范围', False),
             take('/tare/terrain_map_ext', '扩展地形', False),
             take('/sensor_coverage_planner/planner_cloud', '规划表面', False)]
    if not simulation:
        debug.append(take('/tare/status_markers', '状态标签', False))
    else:
        targets[1] = {'Class':'rviz/Pose', 'Name':'下发目标', 'Topic':'/move_base_simple/goal',
                      'Enabled':True, 'Value':True, 'Shape':'Axes', 'Axes Length':.25,
                      'Axes Radius':.025, 'Queue Size':1}
        remap = {'/tare/way_point':'/way_point', '/tare/observed_map':'/scan_map',
                 '/tare/terrain_map':'/terrain_map', '/tare/terrain_map_ext':'/terrain_map_ext',
                 '/tare/display/frontiers':'/sensor_coverage_planner/filtered_frontier_cloud',
                 '/tare/display/uncovered_frontiers':'/sensor_coverage_planner/uncovered_frontier_cloud',
                 '/tare/display/viewpoints':'/sensor_coverage_planner/viewpoint_vis_cloud',
                 '/tare/display/selected_viewpoints':'/sensor_coverage_planner/selected_viewpoint_vis_cloud'}
        for d in targets + progress + maps + debug:
            if d.get('Topic') in remap: d['Topic'] = remap[d['Topic']]
    def group(name, items):
        return {'Class':'rviz/Group', 'Name':name, 'Enabled':True, 'Value':True, 'Displays':items}
    entries = [group('01 目标路径', targets), group('02 探索进度', progress),
               group('03 地图', maps), group('04 调试', debug)]
    displays[:] = [d for d in displays if d.get('Name') != 'TARE']
    displays.append({'Class':'rviz/Group','Name':'TARE','Enabled':algorithm=='tare','Value':algorithm=='tare','Displays':entries})
    for panel in config.get('Panels', []):
        if panel.get('Class') == 'rviz/Displays':
            panel.setdefault('Property Tree Widget', {})['Expanded'] = ['/TARE1', '/TARE1/01 目标路径1', '/TARE1/02 探索进度1']
    return config
