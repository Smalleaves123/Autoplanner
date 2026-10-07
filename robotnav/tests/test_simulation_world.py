from __future__ import annotations

import unittest

import robotnav


class SimulationWorldTests(unittest.TestCase):
    def test_clock_reset_and_deterministic_step(self) -> None:
        world = robotnav.SimulationWorld(
            robotnav.WorldConfig(
                simulation=robotnav.SimulationConfig(dt=0.1), seed=17),
            [robotnav.WorldEntity("robot", robotnav.RobotState(0.0, 0.0))],
        )
        self.assertEqual(world.clock.step_index, 0)
        self.assertAlmostEqual(world.clock.time, 0.0)
        first = world.step({"robot": robotnav.ControlCommand(0.5, 0.0)})
        self.assertEqual(first.clock.step_index, 1)
        self.assertAlmostEqual(first.clock.time, 0.1)
        self.assertEqual(first.state("robot"), first.entities[0].state)
        reset = world.reset([robotnav.WorldEntity(
            "robot", robotnav.RobotState(0.0, 0.0))])
        self.assertEqual(reset.clock.step_index, 0)
        self.assertAlmostEqual(reset.clock.time, 0.0)

    def test_pairwise_collision_observation(self) -> None:
        world = robotnav.SimulationWorld(
            entities=[
                robotnav.WorldEntity("a", robotnav.RobotState(1.0, 1.0)),
                robotnav.WorldEntity("b", robotnav.RobotState(1.2, 1.0)),
            ])
        self.assertEqual(world.observation.collisions, (("a", "b"),))

    def test_differential_drive_accepts_twist_command(self) -> None:
        world = robotnav.SimulationWorld(
            robotnav.WorldConfig(
                simulation=robotnav.SimulationConfig(
                    execution_model="differential_drive",
                    track_width=0.55)),
            [robotnav.WorldEntity("robot", robotnav.RobotState(0.0, 0.0))],
        )
        observation = world.step({
            "robot": robotnav.TwistCommand(0.5, 0.0),
        })
        self.assertGreater(observation.state("robot").x, 0.0)

    def test_unknown_command_entity_is_rejected(self) -> None:
        world = robotnav.SimulationWorld(
            entities=[robotnav.WorldEntity("robot", robotnav.RobotState(0.0, 0.0))])
        with self.assertRaises(KeyError):
            world.step({"ghost": robotnav.ControlCommand(0.0, 0.0)})


if __name__ == "__main__":
    unittest.main()
