#!/usr/bin/env python3
"""Headless physics-backend validation for the local navigation stack.

This intentionally uses a small planar dynamic surrogate rather than a full
vehicle URDF/MJCF. It answers the first integration question: can the same
velocity + steering command stream be executed by MuJoCo and PyBullet while
retaining finite state, actuator response, and contact reporting? The existing
C++ kinematic bicycle remains the fast benchmark backend.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class PhysicsOptions:
    dt: float = 0.05
    wheelbase: float = 1.0
    max_velocity: float = 2.0
    max_acceleration: float = 1.5
    max_deceleration: float = 2.0
    max_steering: float = 0.7
    max_steering_rate: float = 1.5
    velocity_gain: float = 8.0
    yaw_rate_gain: float = 6.0
    substeps: int = 5
    obstacle_rectangles: list[tuple[float, float, float, float]] = field(
        default_factory=list)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def command_for_step(step: int) -> tuple[float, float]:
    if step < 160:
        return 1.0, 0.25
    if step < 320:
        return 1.0, -0.20
    return 0.0, 0.0


class MujocoBicycleSimulator:
    def __init__(self, options: PhysicsOptions):
        import mujoco

        self.mujoco = mujoco
        self.options = options
        xml = f"""
        <mujoco model="robotnav_planar_vehicle">
          <option timestep="{options.dt / options.substeps}" gravity="0 0 0"/>
          <worldbody>
            <geom name="ground" type="plane" size="20 20 0.1"/>
            <geom name="wall" type="box" pos="5 2 0.25"
                  size="0.15 2.0 0.25"/>
            <body name="vehicle" pos="0 0 0.25">
              <joint name="x" type="slide" axis="1 0 0" damping="0.5"/>
              <joint name="y" type="slide" axis="0 1 0" damping="0.5"/>
              <joint name="yaw" type="hinge" axis="0 0 1" damping="0.2"/>
              <geom name="vehicle_geom" type="box"
                    size="0.35 0.25 0.20" mass="12"/>
            </body>
          </worldbody>
        </mujoco>
        """
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "vehicle")
        self.reset()

    def reset(self, x: float = 0.0, y: float = 0.0,
              theta: float = 0.0, velocity: float = 0.0) -> None:
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.qpos[0:3] = (x, y, theta)
        self.data.qvel[0:2] = (
            velocity * math.cos(theta), velocity * math.sin(theta))
        self.mujoco.mj_forward(self.model, self.data)

    def step(self, velocity: float, steering: float) -> dict[str, float]:
        yaw = float(self.data.qpos[2])
        forward_velocity = (
            math.cos(yaw) * float(self.data.qvel[0])
            + math.sin(yaw) * float(self.data.qvel[1])
        )
        target_velocity = clamp(
            velocity, 0.0, self.options.max_velocity)
        target_steering = clamp(
            steering, -self.options.max_steering, self.options.max_steering)
        mass = float(self.model.body_mass[self.body_id])
        force = clamp(
            mass * self.options.velocity_gain *
            (target_velocity - forward_velocity), -40.0, 40.0)
        target_yaw_rate = (
            target_velocity / self.options.wheelbase *
            math.tan(target_steering)
        )
        yaw_torque = clamp(
            self.options.yaw_rate_gain *
            (target_yaw_rate - float(self.data.qvel[2])), -20.0, 20.0)
        self.data.xfrc_applied[self.body_id, :] = (
            force * math.cos(yaw), force * math.sin(yaw), 0.0,
            0.0, 0.0, yaw_torque)
        for _ in range(self.options.substeps):
            self.mujoco.mj_step(self.model, self.data)
        return self.observe()

    def observe(self) -> dict[str, float]:
        yaw = float(self.data.qpos[2])
        forward_velocity = (
            math.cos(yaw) * float(self.data.qvel[0])
            + math.sin(yaw) * float(self.data.qvel[1])
        )
        obstacle_contacts = 0
        wall_geom = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_GEOM, "wall")
        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            if contact.geom1 == wall_geom or contact.geom2 == wall_geom:
                obstacle_contacts += 1
        return {
            "x": float(self.data.qpos[0]),
            "y": float(self.data.qpos[1]),
            "theta": float(self.data.qpos[2]),
            "v": float(forward_velocity),
            "contacts": float(self.data.ncon),
            "obstacle_contacts": float(obstacle_contacts),
        }


class PyBulletBicycleSimulator:
    def __init__(self, options: PhysicsOptions):
        import pybullet as bullet

        self.bullet = bullet
        self.options = options
        self.client = bullet.connect(bullet.DIRECT)
        if self.client < 0:
            raise RuntimeError("PyBullet DIRECT connection failed")
        bullet.setGravity(0.0, 0.0, 0.0, physicsClientId=self.client)
        bullet.setTimeStep(options.dt / options.substeps,
                           physicsClientId=self.client)
        bullet.setPhysicsEngineParameter(
            numSolverIterations=50, physicsClientId=self.client)
        plane_shape = bullet.createCollisionShape(
            bullet.GEOM_PLANE, physicsClientId=self.client)
        bullet.createMultiBody(0, plane_shape, physicsClientId=self.client)
        self.obstacle_ids: list[int] = []
        if options.obstacle_rectangles:
            for center_x, center_y, half_x, half_y in options.obstacle_rectangles:
                shape = bullet.createCollisionShape(
                    bullet.GEOM_BOX,
                    halfExtents=[half_x, half_y, 0.25],
                    physicsClientId=self.client)
                self.obstacle_ids.append(bullet.createMultiBody(
                    0, shape, basePosition=[center_x, center_y, 0.25],
                    physicsClientId=self.client))
        else:
            wall_shape = bullet.createCollisionShape(
                bullet.GEOM_BOX, halfExtents=[0.15, 2.0, 0.25],
                physicsClientId=self.client)
            self.obstacle_ids.append(bullet.createMultiBody(
                0, wall_shape, basePosition=[5.0, 2.0, 0.25],
                physicsClientId=self.client))
        vehicle_shape = bullet.createCollisionShape(
            bullet.GEOM_BOX, halfExtents=[0.35, 0.25, 0.20],
            physicsClientId=self.client)
        self.vehicle = bullet.createMultiBody(
            12.0, vehicle_shape, basePosition=[0.0, 0.0, 0.25],
            physicsClientId=self.client)
        bullet.changeDynamics(
            self.vehicle, -1, linearDamping=0.5, angularDamping=0.2,
            physicsClientId=self.client)

    def reset(self, x: float = 0.0, y: float = 0.0,
              theta: float = 0.0, velocity: float = 0.0) -> None:
        self.bullet.resetBasePositionAndOrientation(
            self.vehicle, [x, y, 0.25],
            self.bullet.getQuaternionFromEuler([0.0, 0.0, theta]),
            physicsClientId=self.client)
        self.bullet.resetBaseVelocity(
            self.vehicle,
            [velocity * math.cos(theta), velocity * math.sin(theta), 0.0],
            [0.0, 0.0, 0.0],
            physicsClientId=self.client)

    def step(self, velocity: float, steering: float) -> dict[str, float]:
        position, quaternion = self.bullet.getBasePositionAndOrientation(
            self.vehicle, physicsClientId=self.client)
        linear_velocity, angular_velocity = self.bullet.getBaseVelocity(
            self.vehicle, physicsClientId=self.client)
        yaw = float(self.bullet.getEulerFromQuaternion(quaternion)[2])
        forward_velocity = (
            math.cos(yaw) * float(linear_velocity[0])
            + math.sin(yaw) * float(linear_velocity[1])
        )
        target_velocity = clamp(
            velocity, 0.0, self.options.max_velocity)
        target_steering = clamp(
            steering, -self.options.max_steering, self.options.max_steering)
        force = clamp(
            12.0 * self.options.velocity_gain *
            (target_velocity - forward_velocity), -40.0, 40.0)
        target_yaw_rate = (
            target_velocity / self.options.wheelbase *
            math.tan(target_steering)
        )
        yaw_torque = clamp(
            self.options.yaw_rate_gain *
            (target_yaw_rate - float(angular_velocity[2])), -20.0, 20.0)
        self.bullet.applyExternalForce(
            self.vehicle, -1,
            [force * math.cos(yaw), force * math.sin(yaw), 0.0],
            list(position), self.bullet.WORLD_FRAME,
            physicsClientId=self.client)
        self.bullet.applyExternalTorque(
            self.vehicle, -1, [0.0, 0.0, yaw_torque],
            self.bullet.WORLD_FRAME, physicsClientId=self.client)
        for _ in range(self.options.substeps):
            self.bullet.stepSimulation(physicsClientId=self.client)
        return self.observe()

    def observe(self) -> dict[str, float]:
        position, quaternion = self.bullet.getBasePositionAndOrientation(
            self.vehicle, physicsClientId=self.client)
        linear_velocity, _ = self.bullet.getBaseVelocity(
            self.vehicle, physicsClientId=self.client)
        yaw = float(self.bullet.getEulerFromQuaternion(quaternion)[2])
        forward_velocity = (
            math.cos(yaw) * float(linear_velocity[0])
            + math.sin(yaw) * float(linear_velocity[1])
        )
        contacts = self.bullet.getContactPoints(
            bodyA=self.vehicle, physicsClientId=self.client)
        obstacle_contacts = sum(
            1 for contact in contacts if contact[2] in self.obstacle_ids)
        return {
            "x": float(position[0]),
            "y": float(position[1]),
            "theta": yaw,
            "v": float(forward_velocity),
            "contacts": float(len(contacts)),
            "obstacle_contacts": float(obstacle_contacts),
        }

    def close(self) -> None:
        if self.client >= 0:
            self.bullet.disconnect(self.client)
            self.client = -1


class PyBulletRacecarSimulator:
    """PyBullet execution backend using its bundled four-wheel racecar URDF."""

    wheel_radius = 0.05
    track_width = 0.20

    def __init__(self, options: PhysicsOptions):
        import pybullet as bullet
        import pybullet_data

        self.bullet = bullet
        self.options = options
        self.client = bullet.connect(bullet.DIRECT)
        if self.client < 0:
            raise RuntimeError("PyBullet DIRECT connection failed")
        bullet.setAdditionalSearchPath(
            pybullet_data.getDataPath(), physicsClientId=self.client)
        bullet.setGravity(0.0, 0.0, -9.81, physicsClientId=self.client)
        bullet.setTimeStep(options.dt / options.substeps,
                           physicsClientId=self.client)
        bullet.setPhysicsEngineParameter(
            numSolverIterations=100, physicsClientId=self.client)
        plane_shape = bullet.createCollisionShape(
            bullet.GEOM_PLANE, physicsClientId=self.client)
        bullet.createMultiBody(0, plane_shape, physicsClientId=self.client)
        self.obstacle_ids: list[int] = []
        if options.obstacle_rectangles:
            for center_x, center_y, half_x, half_y in options.obstacle_rectangles:
                shape = bullet.createCollisionShape(
                    bullet.GEOM_BOX,
                    halfExtents=[half_x, half_y, 0.25],
                    physicsClientId=self.client)
                self.obstacle_ids.append(bullet.createMultiBody(
                    0, shape, basePosition=[center_x, center_y, 0.25],
                    physicsClientId=self.client))
        else:
            wall_shape = bullet.createCollisionShape(
                bullet.GEOM_BOX, halfExtents=[0.15, 2.0, 0.25],
                physicsClientId=self.client)
            self.obstacle_ids.append(bullet.createMultiBody(
                0, wall_shape, basePosition=[5.0, 2.0, 0.25],
                physicsClientId=self.client))
        self.vehicle = bullet.loadURDF(
            "racecar/racecar.urdf", [0.0, 0.0, 0.12],
            useFixedBase=False, physicsClientId=self.client)
        self.joints = {
            bullet.getJointInfo(self.vehicle, index,
                                physicsClientId=self.client)[1].decode(): index
            for index in range(bullet.getNumJoints(
                self.vehicle, physicsClientId=self.client))}
        required = {
            "left_rear_wheel_joint", "right_rear_wheel_joint",
            "left_front_wheel_joint", "right_front_wheel_joint",
            "left_steering_hinge_joint", "right_steering_hinge_joint",
        }
        missing = required - self.joints.keys()
        if missing:
            raise RuntimeError(f"racecar URDF missing joints: {sorted(missing)}")
        self.wheel_joints = [self.joints[name] for name in (
            "left_rear_wheel_joint", "right_rear_wheel_joint",
            "left_front_wheel_joint", "right_front_wheel_joint")]
        self.steering_joints = [self.joints[name] for name in (
            "left_steering_hinge_joint", "right_steering_hinge_joint")]
        self.command_velocity = 0.0
        self.steering = 0.0
        for index in range(bullet.getNumJoints(
                self.vehicle, physicsClientId=self.client)):
            bullet.setJointMotorControl2(
                self.vehicle, index, bullet.VELOCITY_CONTROL,
                targetVelocity=0.0, force=0.0,
                physicsClientId=self.client)
        for index in self.wheel_joints:
            bullet.changeDynamics(
                self.vehicle, index, lateralFriction=1.0,
                rollingFriction=0.01, spinningFriction=0.01,
                physicsClientId=self.client)
        self.reset()

    def reset(self, x: float = 0.0, y: float = 0.0,
              theta: float = 0.0, velocity: float = 0.0) -> None:
        self.bullet.resetBasePositionAndOrientation(
            self.vehicle, [x, y, 0.12],
            self.bullet.getQuaternionFromEuler([0.0, 0.0, theta]),
            physicsClientId=self.client)
        self.bullet.resetBaseVelocity(
            self.vehicle,
            [velocity * math.cos(theta), velocity * math.sin(theta), 0.0],
            [0.0, 0.0, 0.0], physicsClientId=self.client)
        self.command_velocity = max(0.0, velocity)
        self.steering = 0.0
        for index in self.wheel_joints + self.steering_joints:
            self.bullet.resetJointState(
                self.vehicle, index, 0.0, 0.0,
                physicsClientId=self.client)

    def step(self, velocity: float, steering: float) -> dict[str, float]:
        target_velocity = clamp(
            velocity, 0.0, self.options.max_velocity)
        velocity_delta = target_velocity - self.command_velocity
        velocity_limit = (self.options.max_acceleration
                          if velocity_delta >= 0.0
                          else self.options.max_deceleration) * self.options.dt
        self.command_velocity += clamp(
            velocity_delta, -velocity_limit, velocity_limit)
        target_steering = clamp(
            steering, -self.options.max_steering, self.options.max_steering)
        steering_limit = self.options.max_steering_rate * self.options.dt
        self.steering += clamp(
            target_steering - self.steering,
            -steering_limit, steering_limit)
        wheel_velocity = self.command_velocity / self.wheel_radius
        for index in self.wheel_joints:
            self.bullet.setJointMotorControl2(
                self.vehicle, index, self.bullet.VELOCITY_CONTROL,
                targetVelocity=wheel_velocity, force=10.0,
                physicsClientId=self.client)
        for index in self.steering_joints:
            self.bullet.setJointMotorControl2(
                self.vehicle, index, self.bullet.POSITION_CONTROL,
                targetPosition=self.steering, force=5.0,
                positionGain=0.5, velocityGain=1.0,
                physicsClientId=self.client)
        for _ in range(self.options.substeps):
            self.bullet.stepSimulation(physicsClientId=self.client)
        return self.observe()

    def add_static_obstacle(self, center_x: float, center_y: float,
                            half_x: float = 0.5,
                            half_y: float = 0.5) -> int:
        shape = self.bullet.createCollisionShape(
            self.bullet.GEOM_BOX,
            halfExtents=[half_x, half_y, 0.25],
            physicsClientId=self.client)
        body = self.bullet.createMultiBody(
            0, shape, basePosition=[center_x, center_y, 0.25],
            physicsClientId=self.client)
        self.obstacle_ids.append(body)
        return body

    def observe(self) -> dict[str, float]:
        position, quaternion = self.bullet.getBasePositionAndOrientation(
            self.vehicle, physicsClientId=self.client)
        linear_velocity, _ = self.bullet.getBaseVelocity(
            self.vehicle, physicsClientId=self.client)
        yaw = float(self.bullet.getEulerFromQuaternion(quaternion)[2])
        forward_velocity = (
            math.cos(yaw) * float(linear_velocity[0])
            + math.sin(yaw) * float(linear_velocity[1])
        )
        contacts = self.bullet.getContactPoints(
            bodyA=self.vehicle, physicsClientId=self.client)
        obstacle_contacts = sum(
            1 for contact in contacts if contact[2] in self.obstacle_ids)
        return {
            "x": float(position[0]), "y": float(position[1]),
            "theta": yaw, "v": forward_velocity,
            "contacts": float(len(contacts)),
            "obstacle_contacts": float(obstacle_contacts),
        }

    def close(self) -> None:
        if self.client >= 0:
            self.bullet.disconnect(self.client)
            self.client = -1


def run_backend(name: str, steps: int, options: PhysicsOptions) -> dict[str, Any]:
    simulator: Any
    if name == "mujoco":
        simulator = MujocoBicycleSimulator(options)
    elif name == "pybullet":
        simulator = PyBulletBicycleSimulator(options)
    else:
        raise ValueError(f"unknown backend: {name}")

    states = []
    try:
        for step in range(steps):
            velocity, steering = command_for_step(step)
            state = simulator.step(velocity, steering)
            if not all(math.isfinite(value) for value in state.values()):
                raise RuntimeError(f"non-finite state at step {step}: {state}")
            states.append(state)
    finally:
        close = getattr(simulator, "close", None)
        if close is not None:
            close()
    return {
        "backend": name,
        "steps": len(states),
        "final_state": states[-1] if states else {},
        "max_contacts": max((state["contacts"] for state in states), default=0),
        "max_obstacle_contacts": max(
            (state["obstacle_contacts"] for state in states), default=0),
        "max_speed": max((abs(state["v"]) for state in states), default=0),
        "states": states,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco", "pybullet", "both"),
                        default="both")
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--output", type=Path,
                        default=Path("autoplanner/results/physics_smoke.json"))
    args = parser.parse_args()
    if args.steps <= 0:
        raise SystemExit("--steps must be positive")

    backends = ("mujoco", "pybullet") if args.backend == "both" else (args.backend,)
    results = {
        name: run_backend(name, args.steps, PhysicsOptions())
        for name in backends
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(results, indent=2) + "\n")
    for name, result in results.items():
        print(name, result["final_state"],
              "max_contacts=", result["max_contacts"])
    print(f"Physics backend validation written to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
