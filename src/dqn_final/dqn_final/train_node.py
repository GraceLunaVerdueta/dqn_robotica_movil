#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from datetime import datetime
import os

from dqn_project.dqn_agent import DQNAgent
from dqn_project.environment import TurtleBot3Env
from dqn_project.state_processor import StateProcessor


class DQNTrainingNode(Node):

    def __init__(self):
        super().__init__('dqn_training_node')

        self.n_episodes = 500
        self.max_steps_per_episode = 500
        self.state_size = 50
        self.action_size = 5

        self.env = TurtleBot3Env()
        self.state_processor = StateProcessor(n_lidar_bins=48)

        self.agent = DQNAgent(
            state_size=self.state_size,
            action_size=self.action_size,
            learning_rate=0.0008,
            gamma=0.985,
            epsilon=1.0,
            epsilon_min=0.05,
            epsilon_decay=0.995,
            memory_size=50000,
            batch_size=128
        )

        self.episode_rewards = []
        self.success_count = 0

        self.results_dir = os.path.expanduser(
            f"~/dqn_results/results_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        os.makedirs(self.results_dir, exist_ok=True)

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

        for ep in range(self.n_episodes):
            self.env.reset(random_goal=True)

            for _ in range(5):
                rclpy.spin_once(self.env, timeout_sec=0.1)

            state = self.get_processed_state()
            ep_reward = 0.0

            for step in range(self.max_steps_per_episode):
                action = self.agent.act(state, training=True)
                _, r, done = self.env.step(action)
                next_state = self.get_processed_state()

                self.agent.remember(state, action, r, next_state, done)

                if len(self.agent.memory) > self.agent.batch_size:
                    self.agent.replay()

                state = next_state
                ep_reward += r

                if done:
                    break

                rclpy.spin_once(self.env, timeout_sec=0.01)

            self.episode_rewards.append(ep_reward)

            if self.env.is_goal_reached():
                self.success_count += 1

            if ep % 10 == 0:
                avg = np.mean(self.episode_rewards[-10:])
                rate = self.success_count / (ep + 1) * 100
                self.get_logger().info(
                    f"Ep {ep} | R {ep_reward:.1f} | Avg {avg:.1f} | "
                    f"Eps {self.agent.epsilon:.3f} | Success {rate:.1f}%"
                )

            if ep % 50 == 0 and ep > 0:
                self.agent.save(
                    os.path.join(self.results_dir, f"model_ep_{ep}.pkl")
                )

        self.agent.save(os.path.join(self.results_dir, "model_final.pkl"))


def main(args=None):
    rclpy.init(args=args)
    node = DQNTrainingNode()
    try:
        node.train()
    finally:
        node.env.send_velocity(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
