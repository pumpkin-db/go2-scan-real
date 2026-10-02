"""Regression: all-masked softmax must not turn the robot itself into a goal."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
import torch

SOURCE = Path(__file__).resolve().parents[3] / 'algorithms/global_planning/ariadne/src/rl_planner/scripts/agent.py'
tree = ast.parse(SOURCE.read_text())
method = next(m for c in tree.body if isinstance(c, ast.ClassDef) and c.name == 'Agent'
              for m in c.body if isinstance(m, ast.FunctionDef) and m.name == 'select_next_waypoint')
ns = {'torch': torch}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), ns)

class PolicyActionMaskTest(unittest.TestCase):
    def select(self, mask, logits, excluded=None):
        mask = torch.tensor(mask).reshape(1,1,-1)
        scores = torch.tensor(logits,dtype=torch.float32).reshape(1,-1)
        policy = lambda *args: torch.log_softmax(scores.masked_fill(mask[:,0,:] == 1,-1e8),dim=-1)
        agent = SimpleNamespace(policy_net=policy,key_node_coords=np.array([[0.,18.],[2.,18.],[4.,18.]])[:len(logits)])
        observation = [None,None,None,torch.tensor([[[0]]]),torch.arange(len(logits)).reshape(1,-1,1),mask]
        return ns['select_next_waypoint'](agent,observation,excluded_positions=excluded)

    def test_only_self_masked_returns_no_goal(self):
        self.assertEqual(self.select([1],[10.]),(None,None))

    def test_all_masked_returns_no_goal(self):
        self.assertEqual(self.select([1,1],[10.,5.]),(None,None))

    def test_valid_neighbor_still_selected(self):
        point,index=self.select([1,0],[10.,5.])
        self.assertEqual(index,1)
        np.testing.assert_equal(point,[2.,18.])

    def test_blocked_neighbor_does_not_fall_back_to_self(self):
        self.assertEqual(self.select([1,0],[10.,5.],{(2.,18.)}),(None,None))

if __name__ == '__main__':
    unittest.main()
