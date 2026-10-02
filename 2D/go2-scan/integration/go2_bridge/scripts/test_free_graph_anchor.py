"""Real AR graph regressions for an occupied or missing nearest node."""
import ast
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

SCRIPTS = Path(__file__).resolve().parents[3] / 'algorithms/global_planning/ariadne/src/rl_planner/scripts'
sys.path.insert(0, str(SCRIPTS))
import parameter
from utils import get_free_and_connected_map, get_updating_node_coords, is_free, get_frontier_in_map
from node_manager import NodeManager
from agent import Agent

tree = ast.parse((SCRIPTS/'rl_planner.py').read_text())
method = next(m for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='Runner'
              for m in c.body if isinstance(m,ast.FunctionDef) and m.name=='update_planning_graph')
ns = dict(np=np,is_free=is_free,get_updating_node_coords=get_updating_node_coords,
          get_frontier_in_map=get_frontier_in_map)
exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_runner_graph','exec'),ns)

class FreeAnchorTests(unittest.TestCase):
    def setUp(self):
        for name,value in [('CELL_SIZE',.1),('NODE_RESOLUTION',2.),('SENSOR_RANGE',5.),
                           ('UTILITY_RANGE',2.5),('UPDATING_MAP_SIZE',28.)]:
            p=patch.object(parameter,name,value); p.start(); self.addCleanup(p.stop)
        grid=np.full((81,81),parameter.FREE,dtype=int)
        grid[:,65:]=parameter.UNKNOWN
        self.info=SimpleNamespace(map=grid,map_origin_x=0.,map_origin_y=0.,cell_size=.1)

    def test_occupied_seed_never_selects_background(self):
        self.info.map[20,20]=parameter.OCCUPIED
        self.assertFalse(get_free_and_connected_map(np.array([2.,2.]),self.info).any())
        nodes,_=get_updating_node_coords(np.array([2.,2.]),self.info)
        self.assertEqual(nodes.shape,(0,2))

    def test_unknown_and_outside_seed_are_empty(self):
        for p in ([7.,2.],[-1.,2.],[9.,2.]):
            self.assertFalse(get_free_and_connected_map(np.array(p),self.info).any())
            self.assertFalse(is_free(np.array(p),self.info))

    def test_free_seed_keeps_its_component_only(self):
        self.info.map[:,40]=parameter.OCCUPIED
        mask=get_free_and_connected_map(np.array([2.,2.]),self.info)
        self.assertTrue(mask[20,20]); self.assertFalse(mask[:,40:].any())

    def test_invalid_nearest_and_nearby_nodes_are_removed(self):
        manager=NodeManager(np.array([2.,2.]))
        manager.add_node_to_dict(np.array([2.5,2.]),[],None)
        self.info.map[20,20]=self.info.map[20,25]=parameter.OCCUPIED
        manager.check_valid_node(np.array([2.2,2.]),self.info)
        self.assertEqual(len(manager.nodes_dict),0)

    def runner(self,robot=(2.2,2.)):
        agent=Agent(None)
        agent.node_manager=NodeManager(np.array([2.,2.]))
        return SimpleNamespace(robot=agent,robot_location=np.array(robot),map_info=self.info,start=np.array([2.,2.]))

    def test_real_graph_rebuild_after_occupied_anchor_removed(self):
        self.info.map[20,20]=parameter.OCCUPIED
        r=self.runner()
        anchor=ns['update_planning_graph'](r)
        self.assertIsNotNone(anchor)
        self.assertTrue(is_free(anchor,self.info))
        self.assertIsNone(r.robot.node_manager.check_node_exist_in_dict(np.array([2.,2.])))
        self.assertGreater(len(r.robot.node_manager.nodes_dict),1)
        self.assertGreater(len(r.robot.key_neighbor_indices),1)
        self.assertTrue(all(is_free(e.data.coords,self.info) for e in r.robot.node_manager.nodes_dict))

    def test_no_free_start_clears_stale_graph_then_recovers_on_map_update(self):
        r=self.runner(robot=(2.,2.))
        self.info.map[:]=parameter.OCCUPIED
        self.assertIsNone(ns['update_planning_graph'](r))
        self.assertEqual(len(r.robot.key_node_coords),0)
        self.info.map[:,:65]=parameter.FREE
        self.info.map[:,65:]=parameter.UNKNOWN
        self.assertIsNotNone(ns['update_planning_graph'](r))
        self.assertGreater(len(r.robot.key_neighbor_indices),1)

    def test_normal_free_anchor_keeps_original_node(self):
        r=self.runner()
        np.testing.assert_equal(ns['update_planning_graph'](r),[2.,2.])
        self.assertGreater(len(r.robot.key_neighbor_indices),1)

    def test_repeated_updates_keep_forward_actions_and_restore_cleared_node(self):
        self.info.map[20,20]=parameter.OCCUPIED
        r=self.runner()
        for _ in range(5):
            self.assertIsNotNone(ns['update_planning_graph'](r))
            self.assertGreater(len(r.robot.key_neighbor_indices),1)
        self.info.map[20,20]=parameter.FREE
        ns['update_planning_graph'](r)
        self.assertIsNotNone(r.robot.node_manager.check_node_exist_in_dict(np.array([2.,2.])))

if __name__=='__main__':
    unittest.main()
