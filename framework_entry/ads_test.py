import os, sys
from multiprocessing import set_start_method

from utils import parse_args, end
from adv_test_approach import BlackboxOptimization
from adv_test_scenario import ScenarioProblem

class AdvTest():
    def __init__(self, args):
        self.scenario = ScenarioProblem(args)
        self.generator = BlackboxOptimization(args, self.scenario)

    def run(self, ):
        self.generator.fit() # solve
        dir = self.scenario.report() # report
        print(f'Check details in:\n {dir}\n')

if __name__ == '__main__':
    set_start_method('spawn')
    args = parse_args()
    ts = AdvTest(args)
    ts.run()
    end()

