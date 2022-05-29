# encoding: utf-8
from __future__ import print_function
import os
import json
from .core import Scene
from stlib.physics.rigid import Floor, Sphere
import numpy as np


class RandomGoalScene(Scene):
    """This class handles the scene creation of the robot model."""

    def init_scene(self):
        with open(os.path.join(self.asset_dir, "sim_params.json"), 'r') as f:
            self.sim_params = json.load(f)

        # required sim params
        self.dt = self.sim_params['dt']
        self.gravity = self.sim_params['gravity']
        self.with_gui = self.sim_params['with_gui']
        self.debug = self.sim_params['debug']
        self.friction_coef = self.sim_params['friction_coef']
        required_plugins = [
            'SoftRobots',
            'ModelOrderReduction',
        ]

        for i in required_plugins:
            self.root.createObject("RequiredPlugin", name='req_p' + i,
                                   pluginName=i)

        self.root.findData('gravity').value = self.gravity
        self.root.findData('dt').value = self.dt

        # create all the material and cavities
        self.root.createObject('FreeMotionAnimationLoop')
        self.root.createObject('GenericConstraintSolver', printLog='0',
                               tolerance="1e-8", maxIterations="250")

        self.root.createObject('DefaultPipeline', name='collisionPipeline',
                               verbose="0")
        self.root.createObject('BruteForceBroadPhase', name="BP")
        self.root.createObject('BVHNarrowPhase', name="NP")

        self.root.createObject('CollisionResponse', response="FrictionContact", responseParams="mu="+str(self.friction_coef))
        self.root.createObject('LocalMinDistance', name="Proximity",
                               alarmDistance="10.5", contactDistance="0.5",
                               angleCone="0.1")

        self.root.createObject('BackgroundSetting',
                               color='0 0.168627 0.211765')
        self.root.createObject('OglSceneFrame', style="Arrows",
                               alignment="TopRight")
        self.robot.load(self.root)

        Floor(self.root,
              name="Plane",
              translation="0 -0 -0",
              rotation=[90, 0, 0],
              color=[3.0, 0.0, 0.0],
              isAStaticObject=True,
              uniformScale=10)
        Sphere(self.root,
              name="Goal",
              translation="400 -0 -0",
              rotation=[90, 0, 0],
              color=[0.0, 3.0, 0.0],
              isAStaticObject=False,
              uniformScale=10)

    def act(self, action):
        self.robot.act(action)
        """ pos = np.random.randint(100, 400, (2)).tolist()
        rest = self.root.Goal.mstate.position[0][2:]
        pos += rest
        self.root.Goal.mstate.position = [pos] """

    def observe(self):
        ob = self.robot.observe()
        ob['goal_dist'] = self.root.Goal.mstate.position[0][:3]
        return ob
    
    def reset(self):
        """Reset the scene."""
        pos = np.random.randint(-400, 400, (2)).tolist()
        rest = self.root.Goal.mstate.position[0][2:]
        pos += rest
        self.root.Goal.mstate.position = [pos]
        self.root.reset()
        self.robot.reset()
