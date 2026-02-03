#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import os
import matplotlib.patches as patches
import cv2

from dqn_final.dqn_agent import DQNAgent
from dqn_final.environment import TurtleBot3Env
from dqn_final.state_processor import StateProcessor


class DQNTrainingNode(Node):

    def __init__(self):
        super().__init__('dqn_training_node')

        self.n_episodes = 450
        self.max_steps_per_episode = 450
        self.state_size = 50
        self.action_size = 5

        self.env = TurtleBot3Env()
        self.state_processor = StateProcessor(n_lidar_bins=48)
        self.agent = DQNAgent(
            state_size=self.state_size,
            action_size=self.action_size,
            learning_rate=0.001,
            gamma=0.99,
            epsilon=1.0,
            epsilon_min=0.05,
            epsilon_decay=0.99,
            memory_size=50000,
            batch_size=128
        )

        self.episode_rewards = []
        self.episode_steps = []
        self.success_count = 0
        self.collision_count = 0

        self.results_dir = os.path.expanduser(
            f"~/dqn_results/results_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        os.makedirs(self.results_dir, exist_ok=True)

        self.init_debug_window()

    def init_debug_window(self):
        plt.ion()
        self.debug_fig, self.debug_ax = plt.subplots(figsize=(6, 6))
        self.debug_fig.canvas.manager.set_window_title("Live Logic Debugger")

        map_path = '/home/grace/Documents/Diplomado_Robotica/16_01_ws/src/stage_ros2/world/bitmaps/solid_cave.png'
        if os.path.exists(map_path):
            img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
            img = np.flipud(img)
            self.debug_ax.imshow(
                img,
                cmap='gray',
                extent=[-8, 8, -8, 8],
                alpha=0.5,
                origin='lower'
            )

        self.debug_ax.set_xlim(-8, 8)
        self.debug_ax.set_ylim(-8, 8)
        self.debug_ax.grid(True, alpha=0.3)

        self.goal_dot, = self.debug_ax.plot([], [], 'g*', markersize=12)
        self.goal_circle = patches.Circle(
            (0, 0),
            radius=0.5,
            color='g',
            fill=False,
            alpha=0.5
        )
        self.debug_ax.add_patch(self.goal_circle)

    def update_debug_window(self):
        gx_odom, gy_odom = self.env.goal_position
        gx_world = gx_odom + self.env.spawn_x
        gy_world = gy_odom + self.env.spawn_y

        self.goal_dot.set_data([gx_world], [gy_world])
        self.goal_circle.center = (gx_world, gy_world)
        self.goal_circle.radius = self.env.goal_threshold

        self.debug_fig.canvas.draw()
        self.debug_fig.canvas.flush_events()

    def get_processed_state(self):
        return self.state_processor.get_state(
            self.env.scan_data,
            self.env.position,
            self.env.goal_position,
            self.env.yaw
        )

    def train(self):
        while self.env.scan_data is None:
            rclpy.spin_once(self.env, timeout_sec=0.5)

        for episode in range(self.n_episodes):
            self.env.reset(random_goal=True)
            for _ in range(5):
                rclpy.spin_once(self.env, timeout_sec=0.1)

            state = self.get_processed_state()
            episode_reward = 0

            for step in range(self.max_steps_per_episode):
                action = self.agent.act(state, training=True)
                _, reward, done = self.env.step(action)
                next_state = self.get_processed_state()

                self.agent.remember(state, action, reward, next_state, done)

                if len(self.agent.memory) > self.agent.batch_size:
                    self.agent.replay()

                episode_reward += reward
                state = next_state

                if step % 50 == 0:
                    self.update_debug_window()

                if done:
                    break

                rclpy.spin_once(self.env, timeout_sec=0.01)

            self.episode_rewards.append(episode_reward)
            self.episode_steps.append(step + 1)

            if episode % 40 == 0 and episode > 0:
                self.agent.save(
                    os.path.join(self.results_dir, f"model_{episode}.pkl")
                )

        self.agent.save(os.path.join(self.results_dir, "model_final.pkl"))
        plt.close(self.debug_fig)


def main(args=None):
    rclpy.init(args=args)
    trainer = DQNTrainingNode()

    try:
        trainer.train()
    finally:
        trainer.env.send_velocity(0.0, 0.0)
        trainer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
